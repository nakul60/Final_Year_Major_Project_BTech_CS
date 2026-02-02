"""
File operations tool
"""

from typing import Dict
from pathlib import Path
from langchain_core.tools import tool


@tool
def file_operations_tool(operation: str, file_path: str, content: str = "") -> Dict:
    """Perform file operations like read, write, create directories"""
    try:
        path = Path(file_path)

        if operation == "read":
            if path.exists():
                return {"success": True, "content": path.read_text(encoding='utf-8')}
            return {"error": f"File {file_path} not found"}

        elif operation == "write":
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding='utf-8')
            return {"success": True, "message": f"Written to {file_path}"}

        elif operation == "create_dir":
            path.mkdir(parents=True, exist_ok=True)
            return {"success": True, "message": f"Created {file_path}"}

        elif operation == "list":
            if path.exists() and path.is_dir():
                return {"success": True, "files": [str(f) for f in path.iterdir()]}
            return {"error": f"Directory {file_path} not found"}

        return {"error": f"Unknown operation: {operation}"}
    except Exception as e:
        return {"error": str(e)}
