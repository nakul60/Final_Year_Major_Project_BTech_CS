"""
Knowledge base management for embedded systems
"""

from pathlib import Path
from langchain_community.vectorstores import Chroma
from langchain_huggingface import HuggingFaceEmbeddings
from langchain_text_splitters import RecursiveCharacterTextSplitter


class EmbeddedSystemsTools:
    """Collection of tools for embedded systems development"""

    def __init__(self, knowledge_base_path: str = "./knowledge_base"):
        self.knowledge_base_path = Path(knowledge_base_path)
        self.knowledge_base_path.mkdir(exist_ok=True)

        # Initialize embeddings and vector store
        try:
            self.embeddings = HuggingFaceEmbeddings(model_name="all-MiniLM-L6-v2")
            self.vectorstore = Chroma(
                persist_directory=str(self.knowledge_base_path / "chroma_db"),
                embedding_function=self.embeddings
            )
        except Exception as e:
            print(f"⚠️ Vector store initialization failed: {e}")
            self.vectorstore = None

        # Text splitter for documents
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=200
        )

        # Platform configurations
        self.platforms = {
            "arduino": {
                "language": "C++",
                "extensions": [".ino", ".cpp", ".h"],
                "libraries": ["Arduino.h", "SoftwareSerial.h", "Wire.h", "SPI.h"],
            },
            "esp32": {
                "language": "C++",
                "extensions": [".ino", ".cpp", ".h"],
                "libraries": ["WiFi.h", "WebServer.h", "BluetoothSerial.h", "SPIFFS.h"],
            },
            "raspberry_pi": {
                "language": "Python",
                "extensions": [".py"],
                "libraries": ["RPi.GPIO", "gpiozero", "picamera", "spidev", "smbus"],
            }
        }
