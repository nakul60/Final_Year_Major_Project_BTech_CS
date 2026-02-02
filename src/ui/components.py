"""
UI Components - Mermaid renderer and other UI utilities
"""

import re
import streamlit.components.v1 as components


def extract_mermaid(response: str) -> str:
    """Extract Mermaid.js diagram from response"""
    if not response or not isinstance(response, str):
        return None
    match = re.search(r'```mermaid\s*([\s\S]+?)```', response)
    if match:
        return match.group(1).strip()
    return None


def sanitize_mermaid(mermaid_code: str) -> str:
    """Clean and fix common Mermaid syntax issues"""
    if not mermaid_code:
        return ""
    
    mermaid_code = mermaid_code.replace("```mermaid", "").replace("```", "").strip()
    lines = mermaid_code.split('\n')
    fixed_lines = []
    
    for line in lines:
        if not line.strip():
            continue
        
        # Handle graph declaration
        if line.strip().startswith(('graph ', 'flowchart ')):
            graph_match = re.match(r'^(\s*)(graph\s+\w+|flowchart\s+\w+)', line.strip())
            if graph_match:
                fixed_lines.append(graph_match.group(0))
                continue
        
        # Skip or fix subgraph (remove them as they cause issues)
        if 'subgraph' in line.lower() or 'SUBGRAPH' in line:
            continue
        if line.strip().lower() == 'end':
            continue
        
        # Skip comments
        if line.strip().startswith('%%'):
            fixed_lines.append(line)
            continue
        
        # Fix labels in brackets
        def fix_label(match):
            label = match.group(1)
            for old, new in {'Ω': 'Ohm', 'µ': 'u', '°': ' deg', ':': ' - ', '"': "'", 
                           '(': ' ', ')': ' ', '<': ' ', '>': ' ', '{': ' ', '}': ' ', 
                           '#': 'No', '&': 'and', '|': ' '}.items():
                label = label.replace(old, new)
            # Clean up multiple spaces
            cleaned_label = re.sub(r'\s+', ' ', label).strip()
            return f'[{cleaned_label}]'
        
        line = re.sub(r'\[([^\]]+)\]', fix_label, line)
        
        # Fix node IDs - only alphanumeric and underscore
        def fix_node_id(match):
            before = match.group(1) or ''
            node_id = match.group(2)
            after = match.group(3)
            clean_id = re.sub(r'[^a-zA-Z0-9_]', '_', node_id)
            clean_id = re.sub(r'_+', '_', clean_id).strip('_')
            if clean_id and clean_id[0].isdigit():
                clean_id = 'N' + clean_id
            return f'{before}{clean_id or "Node"}{after}'
        
        line = re.sub(r'(^|\s)([a-zA-Z0-9_:]+)(\[)', fix_node_id, line)
        line = re.sub(r'(^|\s)([a-zA-Z0-9_:]+)(\s*-->)', fix_node_id, line)
        line = re.sub(r'(-->?\s*)([a-zA-Z0-9_:]+)(\[|\s*$)', fix_node_id, line)
        
        # Remove colons from IDs
        line = line.replace(':', '_')
        
        if line.strip():
            fixed_lines.append(line)
    
    result = '\n'.join(fixed_lines)
    if not any(result.strip().startswith(x) for x in ['graph ', 'flowchart ']):
        result = 'graph TD\n' + result
    
    return result


def render_mermaid(mermaid_code: str, height: int = 450):
    """Render Mermaid diagram using HTML/JS component"""
    if not mermaid_code:
        return
    
    mermaid_code = sanitize_mermaid(mermaid_code)
    mermaid_code_js = mermaid_code.replace("\\", "\\\\").replace("`", "\\`").replace("$", "\\$")
    
    html_code = f"""
    <div id="mermaid-container" style="background: white; padding: 15px; border-radius: 8px; border: 1px solid #ddd;">
        <div id="mermaid-output"></div>
        <div id="mermaid-error" style="color: #c00; display: none; padding: 10px; background: #fff0f0; border-radius: 4px; margin-top: 10px; font-size: 12px;"></div>
        <details id="mermaid-source" style="margin-top: 10px; display: none;">
            <summary style="cursor: pointer; color: #666;">Show Source</summary>
            <pre style="background: #f5f5f5; padding: 10px; font-size: 11px; overflow-x: auto;">{mermaid_code}</pre>
        </details>
    </div>
    <script src="https://cdn.jsdelivr.net/npm/mermaid@10.6.1/dist/mermaid.min.js"></script>
    <script>
        (async function() {{
            mermaid.initialize({{ startOnLoad: false, theme: 'default', securityLevel: 'loose' }});
            try {{
                const {{ svg }} = await mermaid.render('mermaid-svg', `{mermaid_code_js}`);
                document.getElementById('mermaid-output').innerHTML = svg;
            }} catch (err) {{
                document.getElementById('mermaid-error').style.display = 'block';
                document.getElementById('mermaid-error').innerHTML = '⚠️ Diagram error: ' + err.message;
                document.getElementById('mermaid-source').style.display = 'block';
            }}
        }})();
    </script>
    """
    components.html(html_code, height=height, scrolling=True)
