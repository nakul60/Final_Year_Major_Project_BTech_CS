import streamlit as st
import asyncio
import os
import shutil
from pathlib import Path
from dotenv import load_dotenv
from agent6 import EmbeddedSystemsAgent

# Load environment variables from .env file
load_dotenv()
GROQ_API_KEY = os.getenv("GROQ_API_KEY")

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
        if not GROQ_API_KEY:
            st.error("GROQ_API_KEY not found in .env file!")
        else:
            st.session_state["agent"] = EmbeddedSystemsAgent(GROQ_API_KEY)
            st.session_state["platform"] = platform
            st.success("Agent initialized!")

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
                    st.session_state.messages.append({"role": "assistant", "content": response})
                else:
                    st.error(result["error"])

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
                st.download_button("📥 Download Code", data=code_text, file_name=f"generated_code.{ext}")
            else:
                st.error(result["error"])

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