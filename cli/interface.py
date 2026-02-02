"""
CLI Interface for Embedded Systems AI Agent
"""

import asyncio
from datetime import datetime
from pathlib import Path
from src.agent import EmbeddedSystemsAgent


class EmbeddedSystemsCLI:
    """Enhanced CLI interface with LangGraph integration"""

    def __init__(self, api_key: str):
        self.agent = EmbeddedSystemsAgent(api_key)
        self.current_platform = ""
        self.session_history = []

    async def run_interactive_session(self):
        """Run enhanced interactive session"""
        print("🤖 Embedded Systems AI Agent")
        print("=" * 50)
        print("Commands: chat, generate, project, search, knowledge, tools, platform, history, help, quit")
        print("=" * 50)

        while True:
            try:
                command = input(f"\n[{self.current_platform or 'general'}] > ").strip().lower()

                if command == "quit":
                    print("👋 Goodbye!")
                    break
                elif command == "chat":
                    await self._handle_chat()
                elif command == "generate":
                    await self._handle_generate()
                elif command == "project":
                    await self._handle_project()
                elif command == "search":
                    await self._handle_search()
                elif command == "knowledge":
                    await self._handle_knowledge()
                elif command == "tools":
                    self._show_tools()
                elif command == "platform":
                    self._set_platform()
                elif command == "history":
                    self._show_history()
                elif command == "help":
                    self._show_help()
                else:
                    print("❓ Unknown command. Type 'help' for available commands.")

            except KeyboardInterrupt:
                print("\n👋 Exiting...")
                break
            except Exception as e:
                print(f"❌ Error: {e}")

    async def _handle_chat(self):
        question = input("Ask me anything: ").strip()
        if not question:
            return
        print("🤔 Thinking...")
        result = await self.agent.process_request(question, self.current_platform)
        if result["success"]:
            print(f"\n💡 {result['response']}")
            self.session_history.append({"type": "chat", "question": question, "timestamp": result['timestamp']})
        else:
            print(f"❌ {result['error']}")

    async def _handle_generate(self):
        platform = self.current_platform or input("Platform (arduino/esp32/raspberry_pi): ").strip().lower()
        requirements = input("Describe what you want: ").strip()
        if not requirements:
            return
        print("⚡ Generating...")
        result = await self.agent.process_request(f"Generate {platform} code for: {requirements}", platform)
        if result["success"]:
            print(f"\n✅ Generated:\n{'='*50}\n{result['response']}\n{'='*50}")
            self.session_history.append({"type": "generate", "platform": platform, "timestamp": result['timestamp']})
        else:
            print(f"❌ {result['error']}")

    async def _handle_project(self):
        platform = self.current_platform or input("Platform: ").strip().lower()
        name = input("Project name: ").strip()
        requirements = input("Requirements: ").strip()
        if not name or not requirements:
            print("⚠️ Name and requirements required")
            return
        print("🏗️ Creating project...")
        result = await self.agent.generate_project(platform, requirements, name)
        if result["success"]:
            print(f"✅ Created: {result['project_path']}")
            print(f"📄 Files: {', '.join(result['files_created'])}")
        else:
            print(f"❌ {result['error']}")

    async def _handle_search(self):
        query = input("Search: ").strip()
        if query:
            print("🔍 Searching...")
            result = await self.agent.process_request(f"Search for: {query}")
            print(result.get('response', result.get('error', 'No results')))

    async def _handle_knowledge(self):
        file_path = input("File path (PDF/TXT): ").strip()
        if file_path:
            success = await self.agent.add_knowledge(file_path)
            print("✅ Added!" if success else "❌ Failed")

    def _show_tools(self):
        print("\n🔧 Available Tools:")
        tools = ["web_search", "component_lookup", "pinout_lookup", "code_template", "code_validator", "library_lookup", "file_operations"]
        for t in tools:
            print(f"  - {t}")

    def _set_platform(self):
        platform = input("Platform (arduino/esp32/raspberry_pi/clear): ").strip().lower()
        if platform == "clear":
            self.current_platform = ""
            print("✅ Cleared")
        elif platform in ["arduino", "esp32", "raspberry_pi"]:
            self.current_platform = platform
            print(f"✅ Set to {platform}")
        else:
            print("❌ Invalid")

    def _show_history(self):
        if not self.session_history:
            print("📝 No history")
            return
        print(f"\n📝 History ({len(self.session_history)} items):")
        for item in self.session_history[-5:]:
            print(f"  [{item['type']}] {item.get('timestamp', '')}")

    def _show_help(self):
        print("""
🤖 Commands:
  chat      - Ask questions
  generate  - Generate code
  project   - Create full project
  search    - Web search
  knowledge - Add documents
  tools     - List tools
  platform  - Set platform
  history   - View history
  help      - Show this
  quit      - Exit
""")
