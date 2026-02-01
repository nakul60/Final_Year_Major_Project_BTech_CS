import streamlit as st
import streamlit.components.v1 as components
import asyncio
import os
import shutil
from pathlib import Path
from dotenv import load_dotenv
from agent_temp import EmbeddedSystemsAgent, power_profile_estimator_fn

# Load environment variables from .env file
load_dotenv()
API_KEY = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")

# Streamlit page config
st.set_page_config(page_title="Embedded Systems AI Agent", layout="wide")
st.title("🤖 Embedded Systems AI Agent")

# Initialize session state
if "agent" not in st.session_state:
    st.session_state["agent"] = None
if "platform" not in st.session_state:
    st.session_state["platform"] = ""
if "projects_list" not in st.session_state:
    st.session_state["projects_list"] = []
if "messages" not in st.session_state:
    st.session_state.messages = []

# Sidebar: Configuration and Projects
with st.sidebar:
    st.header("⚙️ Configuration")
    platform = st.selectbox("Select Platform", ["", "arduino", "esp32", "raspberry_pi"], index=0)
    
    if st.button("Initialize Agent"):
        if not API_KEY:
            st.error("API key not found! Please set GEMINI_API_KEY in your .env file.")
        else:
            st.session_state["agent"] = EmbeddedSystemsAgent(API_KEY)
            st.session_state["platform"] = platform
            st.success("Agent initialized! 🚀")

    st.markdown("---")
    st.subheader("📁 Projects")
    projects_path = Path("./knowledge_base/projects")
    projects_list = []
    if projects_path.exists():
        projects_list = [p.name for p in projects_path.iterdir() if p.is_dir()]
        st.session_state["projects_list"] = projects_list
    selected_project = st.selectbox("Select a project", [""] + projects_list)

# Ensure agent is initialized
if not st.session_state["agent"]:
    st.warning("Please initialize the agent first.")
    st.stop()

agent = st.session_state["agent"]
st.session_state["platform"] = platform

# Utility to extract Mermaid.js diagram from response
import re
def extract_mermaid(response):
    if not response or not isinstance(response, str):
        return None
    match = re.search(r'```mermaid\s*([\s\S]+?)```', response)
    if match:
        return match.group(1).strip()
    return None

def sanitize_mermaid(mermaid_code):
    """Clean and fix common Mermaid syntax issues for robust rendering"""
    if not mermaid_code:
        return ""
    
    # Remove any extra backticks or markdown formatting
    mermaid_code = mermaid_code.replace("```mermaid", "").replace("```", "").strip()
    
    lines = mermaid_code.split('\n')
    fixed_lines = []
    
    for line in lines:
        original_line = line
        
        # Skip empty lines
        if not line.strip():
            continue
            
        # Skip graph declaration lines
        if line.strip().startswith(('graph', 'flowchart', 'subgraph', 'end', '%%')):
            fixed_lines.append(line)
            continue
        
        # Fix node labels in square brackets - handle special characters
        def fix_label(match):
            label = match.group(1)
            # Replace problematic characters with safe alternatives
            label = label.replace('Ω', 'Ohm')  # Omega symbol
            label = label.replace('Ω', 'Ohm')  # Unicode Omega
            label = label.replace('µ', 'u')     # Micro symbol
            label = label.replace('°', ' deg')  # Degree symbol
            label = label.replace(':', ' ')     # Colons cause issues
            label = label.replace('"', "'")     # Double quotes
            label = label.replace('(', ' ')     # Parentheses can cause issues
            label = label.replace(')', ' ')
            label = label.replace('<', ' ')     # Angle brackets
            label = label.replace('>', ' ')
            label = label.replace('{', ' ')     # Curly braces
            label = label.replace('}', ' ')
            label = label.replace('#', 'No')    # Hash symbol
            label = label.replace('&', 'and')   # Ampersand
            # Clean up multiple spaces
            label = re.sub(r'\s+', ' ', label).strip()
            return f'[{label}]'
        
        line = re.sub(r'\[([^\]]+)\]', fix_label, line)
        
        # Fix node IDs - ensure they only contain valid characters (letters, numbers, underscores)
        # This handles cases like "R1[10k Ohm Resistor]" where R1 is the ID
        def fix_node_id(match):
            before = match.group(1) if match.group(1) else ''
            node_id = match.group(2)
            after = match.group(3)
            # Clean node ID - only alphanumeric and underscore
            clean_id = re.sub(r'[^a-zA-Z0-9_]', '_', node_id)
            # Ensure ID doesn't start with a number
            if clean_id and clean_id[0].isdigit():
                clean_id = 'N' + clean_id
            return f'{before}{clean_id}{after}'
        
        # Fix node IDs before brackets or arrows
        line = re.sub(r'(^|\s)([a-zA-Z0-9_:]+)(\[)', fix_node_id, line)
        line = re.sub(r'(^|\s)([a-zA-Z0-9_:]+)(\s*-->)', fix_node_id, line)
        line = re.sub(r'(^|\s)([a-zA-Z0-9_:]+)(\s*---)', fix_node_id, line)
        line = re.sub(r'(-->?\s*)([a-zA-Z0-9_:]+)(\[|\s*$)', fix_node_id, line)
        
        # Fix arrow labels - ensure proper syntax |label|
        line = re.sub(r'-->\|([^|]+)\|\s*', r'--> |\1| ', line)
        line = re.sub(r'---\|([^|]+)\|\s*', r'--- |\1| ', line)
        
        # Remove any remaining colons in node IDs (outside of labels)
        # Split by arrows and fix each part
        parts = re.split(r'(-->|---)', line)
        cleaned_parts = []
        for part in parts:
            if part in ('-->', '---'):
                cleaned_parts.append(part)
            else:
                # Don't touch content inside brackets, but fix IDs outside
                if '[' in part:
                    idx = part.index('[')
                    before_bracket = part[:idx].replace(':', '_')
                    after_bracket = part[idx:]
                    cleaned_parts.append(before_bracket + after_bracket)
                else:
                    cleaned_parts.append(part.replace(':', '_'))
        line = ''.join(cleaned_parts)
        
        fixed_lines.append(line)
    
    return '\n'.join(fixed_lines)

def render_mermaid(mermaid_code):
    """Render Mermaid diagram using HTML/JS component with better error handling"""
    if not mermaid_code:
        return
    
    # Sanitize the mermaid code
    mermaid_code = sanitize_mermaid(mermaid_code)
    
    # Escape for JavaScript string (but keep mermaid syntax intact)
    mermaid_code_js = mermaid_code.replace("\\", "\\\\").replace("`", "\\`").replace("$", "\\$")
    
    # HTML template with Mermaid.js and proper error handling
    html_code = f"""
    <div id="mermaid-container" style="background-color: white; padding: 15px; border-radius: 8px; border: 1px solid #ddd; min-height: 100px;">
        <div id="mermaid-output"></div>
        <div id="mermaid-error" style="color: #c00; display: none; padding: 10px; background: #fff0f0; border-radius: 4px; margin-top: 10px; font-family: monospace; font-size: 12px;"></div>
        <details id="mermaid-source" style="margin-top: 10px; display: none;">
            <summary style="cursor: pointer; color: #666;">Show Mermaid Source</summary>
            <pre style="background: #f5f5f5; padding: 10px; border-radius: 4px; overflow-x: auto; font-size: 11px;">{mermaid_code}</pre>
        </details>
    </div>
    <script src="https://cdn.jsdelivr.net/npm/mermaid@10.6.1/dist/mermaid.min.js"></script>
    <script>
        (async function() {{
            const mermaidCode = `{mermaid_code_js}`;
            const outputDiv = document.getElementById('mermaid-output');
            const errorDiv = document.getElementById('mermaid-error');
            const sourceDiv = document.getElementById('mermaid-source');
            
            mermaid.initialize({{ 
                startOnLoad: false,
                theme: 'default',
                securityLevel: 'loose',
                flowchart: {{
                    useMaxWidth: true,
                    htmlLabels: true,
                    curve: 'linear'
                }}
            }});
            
            try {{
                const {{ svg }} = await mermaid.render('mermaid-svg', mermaidCode);
                outputDiv.innerHTML = svg;
            }} catch (err) {{
                console.error('Mermaid error:', err);
                errorDiv.style.display = 'block';
                errorDiv.innerHTML = '⚠️ Diagram rendering failed. Error: ' + (err.message || String(err));
                sourceDiv.style.display = 'block';
            }}
        }})();
    </script>
    """
    components.html(html_code, height=450, scrolling=True)

# Tabs
tab_chat, tab_generate, tab_project, tab_knowledge, tab_history = st.tabs(
    ["💬 Chat", "⚡ Generate Code", "🏗️ Project Builder", "📚 Knowledge Base", "📝 Debug Info"]
)

# ------------------ Chat Tab ------------------
with tab_chat:
    st.subheader("💬 Embedded Systems Assistant")
    
    # Display chat history
    for message in st.session_state.messages:
        with st.chat_message(message["role"]):
            st.markdown(message["content"])

    # Chat Input
    if prompt := st.chat_input("Ask about pinouts, protocols, or sensors..."):
        with st.chat_message("user"):
            st.markdown(prompt)
        st.session_state.messages.append({"role": "user", "content": prompt})

        with st.chat_message("assistant"):
            with st.spinner("Thinking..."):
                result = asyncio.run(agent.process_request(prompt, platform))
                if result["success"]:
                    response = result["response"]
                    st.markdown(response)
                    # Show Mermaid diagram if present
                    mermaid = extract_mermaid(response)
                    if mermaid:
                        st.markdown("**Wiring Diagram:**")
                        render_mermaid(mermaid)
                    st.session_state.messages.append({"role": "assistant", "content": response})
                else:
                    error_msg = result["error"]
                    # Show different UI for rate limit errors
                    if "rate limit" in error_msg.lower() or "quota" in error_msg.lower():
                        st.warning(f"⏳ {error_msg}")
                        st.info("💡 **Tip:** The free tier of Gemini 2.5 Flash allows 20 requests/day. Consider upgrading to a paid plan for higher limits.")
                    else:
                        st.error(error_msg)

# ------------------ Code Generation Tab ------------------
with tab_generate:
    st.subheader("⚡ Quick Code Snippet")
    requirements = st.text_area("What should the code do?", placeholder="e.g. Read I2C BME280 and log to Serial")
    if st.button("Generate Code", key="generate_btn"):
        if requirements:
            with st.spinner("Generating..."):
                result = asyncio.run(agent.process_request(f"Generate {platform} code for: {requirements}", platform))
            if result["success"]:
                code_text = result["response"]
                lang = "cpp" if platform in ["arduino", "esp32"] else "python"
                ext = "ino" if lang == "cpp" else "py"
                st.code(code_text, language=lang)
                # Show Mermaid diagram if present
                mermaid = extract_mermaid(code_text)
                if mermaid:
                    st.markdown("**Wiring Diagram:**")
                    render_mermaid(mermaid)
                # Power profile estimation
                try:
                    power_report = power_profile_estimator_fn(code_text, platform)
                    if power_report:
                        st.markdown("**⚡ Power Consumption Estimate:**")
                        st.json(power_report)
                except Exception as e:
                    st.warning(f"Power estimation unavailable: {e}")
                st.download_button("📥 Download Code", data=code_text, file_name=f"generated_code.{ext}")
            else:
                error_msg = result["error"]
                if "rate limit" in error_msg.lower() or "quota" in error_msg.lower():
                    st.warning(f"⏳ {error_msg}")
                    st.info("💡 **Tip:** The free tier of Gemini 2.5 Flash allows 20 requests/day. Consider upgrading to a paid plan for higher limits.")
                else:
                    st.error(error_msg)

# ------------------ Project Builder Tab ------------------
with tab_project:
    st.subheader("🏗️ Full Project Workspace")
    col1, col2 = st.columns(2)
    with col1:
        p_name = st.text_input("Project Name", placeholder="Smart_Garden")
    with col2:
        st.info(f"Target Platform: {platform}")

    p_reqs = st.text_area("Detailed Requirements", placeholder="List sensors, display type, and logic...")
    
    if st.button("Build Project Structure", key="project_btn"):
        if p_name and p_reqs:
            with st.status("🏗️ Building project...", expanded=True) as status:
                result = asyncio.run(agent.generate_project(platform, p_reqs, p_name))
                if result["success"]:
                    status.update(label="✅ Project Built!", state="complete")
                    
                    # Create ZIP for user download
                    proj_dir = Path(f"./knowledge_base/projects/{p_name}")
                    if proj_dir.exists():
                        shutil.make_archive(p_name, 'zip', proj_dir)
                        with open(f"{p_name}.zip", "rb") as f:
                            st.download_button(f"📦 Download {p_name}.zip", f, file_name=f"{p_name}.zip")
                else:
                    status.update(label="❌ Build Failed", state="error")
                    st.error(result["error"])

# ------------------ Knowledge Base Tab ------------------
with tab_knowledge:
    st.subheader("📚 Datasheet & Docs Indexer")
    uploaded_file = st.file_uploader("Upload PDF/TXT to enhance Agent knowledge", type=["pdf", "txt"])
    
    if uploaded_file:
        # Temporary save
        temp_path = Path(f"temp_{uploaded_file.name}")
        with open(temp_path, "wb") as f:
            f.write(uploaded_file.getbuffer())
        
        with st.spinner("Analyzing document..."):
            success = asyncio.run(agent.add_knowledge(str(temp_path)))
            if success:
                st.success(f"Successfully indexed {uploaded_file.name}!")
            else:
                st.error("Indexing failed.")
        
        # Cleanup temp file
        if temp_path.exists():
            os.remove(temp_path)

# ------------------ History Tab (Debug) ------------------
with tab_history:
    st.subheader("📝 Session Debugger")
    if st.button("Clear Chat History"):
        st.session_state.messages = []
        st.rerun()
    st.json({"Platform": platform, "Projects Found": st.session_state["projects_list"]})

# ------------------ Project Viewer ------------------
if selected_project:
    st.divider()
    st.subheader(f"📂 Project View: {selected_project}")
    project_dir = projects_path / selected_project

    # Load documentation
    readme_file = project_dir / "README.md"
    if readme_file.exists():
        with st.expander("📄 Documentation", expanded=True):
            st.markdown(readme_file.read_text())

    # Find and show code
    for ext in [".py", ".ino", ".cpp"]:
        code_path = project_dir / f"{selected_project}{ext}"
        if code_path.exists():
            with st.expander("💻 Source Code"):
                st.code(code_path.read_text(), language="cpp" if ext != ".py" else "python")