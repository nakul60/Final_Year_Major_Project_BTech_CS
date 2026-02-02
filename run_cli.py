#!/usr/bin/env python3
"""
CLI Runner for Embedded Systems AI Agent
Run this script to start the interactive CLI interface.

Usage:
    python run_cli.py
"""

import os
import sys
import asyncio
from pathlib import Path
from dotenv import load_dotenv

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

from cli.interface import EmbeddedSystemsCLI


def main():
    """Main entry point for CLI"""
    # Load environment variables
    load_dotenv()
    api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
    
    if not api_key:
        print("❌ Error: No API key found!")
        print("Please set GEMINI_API_KEY or GOOGLE_API_KEY in your .env file")
        print("\nExample .env file:")
        print("  GEMINI_API_KEY=your_api_key_here")
        sys.exit(1)
    
    print("🚀 Starting Embedded Systems AI Agent CLI...")
    print("-" * 50)
    
    try:
        cli = EmbeddedSystemsCLI(api_key)
        asyncio.run(cli.run_interactive_session())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
