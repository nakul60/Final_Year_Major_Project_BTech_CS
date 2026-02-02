# 🤖 Generative AI Copilot for Embedded Software

![Python](https://img.shields.io/badge/Python-3.11+-blue?logo=python&logoColor=white)
![Streamlit](https://img.shields.io/badge/Streamlit-1.28+-red?logo=streamlit&logoColor=white)
![LangChain](https://img.shields.io/badge/LangChain-0.1+-green?logo=chainlink&logoColor=white)
![LangGraph](https://img.shields.io/badge/LangGraph-Enabled-purple)
![Google Gemini](https://img.shields.io/badge/Gemini-2.5_Flash-blue?logo=google&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-yellow)

> **Final Year Major Project** - An AI-powered assistant for embedded systems development supporting Arduino, ESP32, and Raspberry Pi platforms.

---

## ✨ Features

- 🔧 **Multi-Platform Support**: Arduino, ESP32, Raspberry Pi
- 💬 **Intelligent Chat**: Ask questions about pinouts, protocols, sensors, and more
- ⚡ **Code Generation**: Auto-generate boilerplate code for sensors, actuators, and communication protocols
- 🏗️ **Project Builder**: Create complete project structures with code, wiring diagrams, and documentation
- 📚 **Knowledge Base**: RAG-powered system with embedded PDF documentation
- 🔍 **Web Search**: Real-time information lookup via DuckDuckGo
- 📊 **Power Estimation**: Estimate power consumption for your projects
- ✅ **Code Validation**: Syntax checking and best practices verification

---

## 📁 Project Structure

```
EmbeddedAgent/
├── app.py                    # 🎯 Streamlit web application (main entry)
├── run_cli.py                # 🖥️ CLI application entry point
├── requirements.txt          # 📦 Python dependencies
├── .env                      # 🔐 Environment variables (API keys)
│
├── src/                      # 📂 Source modules
│   ├── agent/                # 🤖 Core AI agent
│   │   ├── core.py           #    LangGraph-based agent implementation
│   │   ├── state.py          #    State management for agent workflow
│   │   ├── prompts.py        #    System prompts and templates
│   │   └── __init__.py
│   │
│   ├── tools/                # 🔧 Agent tools
│   │   ├── web_search.py     #    DuckDuckGo web search tool
│   │   ├── component_lookup.py   Component datasheets lookup
│   │   ├── pinout_lookup.py  #    Pinout reference tool
│   │   ├── code_templates.py #    Code template generator
│   │   ├── code_validator.py #    Syntax & best practices checker
│   │   ├── library_lookup.py #    Library recommendations
│   │   ├── file_operations.py    File read/write operations
│   │   ├── power_estimator.py    Power consumption calculator
│   │   └── __init__.py
│   │
│   ├── hardware/             # 🔌 Hardware abstraction
│   │   ├── registry.py       #    Hardware component registry
│   │   ├── verifier.py       #    Hardware compatibility checker
│   │   └── __init__.py
│   │
│   ├── knowledge/            # 📖 Knowledge management
│   │   ├── manager.py        #    ChromaDB-based RAG system
│   │   └── __init__.py
│   │
│   ├── ui/                   # 🎨 UI components
│   │   ├── components.py     #    Mermaid diagrams, helpers
│   │   └── __init__.py
│   │
│   └── __init__.py
│
├── cli/                      # 🖥️ CLI interface
│   ├── interface.py          #    Interactive command-line interface
│   └── __init__.py
│
├── knowledge_base/           # 📚 Knowledge storage
│   ├── *.pdf                 #    Reference PDFs (Arduino, ESP32, RPi)
│   ├── chroma_db/            #    Vector database storage
│   └── projects/             #    Generated project files
│
└── Screenshots/              # 📸 Application screenshots
```

---

## 🚀 Quick Start

### Prerequisites

- Python 3.11+
- Google Gemini API key ([Get one free](https://makersuite.google.com/app/apikey))

### Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd EmbeddedAgent
   ```

2. **Create virtual environment**
   ```bash
   python -m venv .venv
   
   # Windows
   .\.venv\Scripts\activate
   
   # Linux/Mac
   source .venv/bin/activate
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

4. **Configure environment**
   ```bash
   # Create .env file
   echo "GEMINI_API_KEY=your_api_key_here" > .env
   ```

### Running the Application

#### 🌐 Web Interface (Streamlit)
```bash
streamlit run app.py
```
Open http://localhost:8501 in your browser.

#### 🖥️ Command Line Interface
```bash
python run_cli.py
```

---

## 🎯 Usage Examples

### Chat Assistant
Ask questions about embedded systems:
- "What are the I2C pins on ESP32?"
- "How do I connect a DHT11 sensor to Arduino?"
- "Explain SPI communication protocol"

### Code Generation
Generate code for specific tasks:
- "Generate Arduino code for reading DS18B20 temperature sensor"
- "Create ESP32 WiFi web server code"
- "Write Python code for Raspberry Pi GPIO LED blink"

### Project Builder
Create complete projects:
- Platform: ESP32
- Name: WeatherStation
- Requirements: "Read DHT22 sensor and display on OLED with WiFi data logging"

---

## 🔧 Configuration

### Environment Variables

| Variable | Description | Required |
|----------|-------------|----------|
| `GEMINI_API_KEY` | Google Gemini API key | Yes |
| `GOOGLE_API_KEY` | Alternative API key name | No |

### Supported Platforms

| Platform | Description |
|----------|-------------|
| `arduino` | Arduino Uno, Mega, Nano, etc. |
| `esp32` | ESP32, ESP32-C3, ESP32-S3 |
| `raspberry_pi` | Raspberry Pi (Python/GPIO) |

---

## 📸 Screenshots

| Dashboard |
|:--------:|
| ![Dashboard](Screenshots/image.png) |

---

## 🛠️ Tech Stack

- **LLM**: Google Gemini 2.5 Flash
- **Framework**: LangChain + LangGraph
- **Vector DB**: ChromaDB
- **Web UI**: Streamlit
- **Document Processing**: PyPDF, LangChain loaders

---

## 📝 CLI Commands

| Command | Description |
|---------|-------------|
| `chat` | Ask questions interactively |
| `generate` | Generate code for a platform |
| `project` | Create a full project structure |
| `search` | Search the web for information |
| `knowledge` | Add documents to knowledge base |
| `tools` | List available tools |
| `platform` | Set/change current platform |
| `history` | View session history |
| `help` | Show help menu |
| `quit` | Exit the CLI |

---

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 👨‍💻 Author

**Nakul Arora, Abhishek Chavan** - Final Year Major Project

---

## ⚠️ Notes

- Free tier Gemini API: ~20 requests/day limit
- For heavy usage, consider upgrading to paid API tier
- Knowledge base PDFs are loaded on first run (may take time)

---

*Built with ❤️ for Edge AI  enthusiasts*