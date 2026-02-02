"""
Code validator for embedded platforms
"""

from typing import Dict
from langchain_core.tools import tool


@tool
def code_validator_tool(code: str, platform: str) -> Dict:
    """Validate code syntax and structure for embedded platforms"""
    try:
        result = {"success": True, "warnings": [], "errors": [], "suggestions": []}

        if platform.lower() in ["arduino", "esp32"]:
            if "setup()" not in code:
                result["errors"].append("Missing setup() function")
            if "loop()" not in code:
                result["warnings"].append("Missing loop() function")
            if "Serial.begin" in code and "#include <Arduino.h>" not in code:
                result["suggestions"].append("Consider adding #include <Arduino.h>")
            if "WiFi." in code and "#include <WiFi.h>" not in code:
                result["errors"].append("Using WiFi without #include <WiFi.h>")

        elif platform.lower() == "raspberry_pi":
            try:
                compile(code, '<string>', 'exec')
            except SyntaxError as e:
                result["errors"].append(f"Python syntax error: {e}")
            if "GPIO." in code and "import RPi.GPIO" not in code:
                result["warnings"].append("Using GPIO without importing RPi.GPIO")
            if "GPIO.cleanup()" not in code and "GPIO." in code:
                result["suggestions"].append("Add GPIO.cleanup() for proper cleanup")

        if result["errors"]:
            result["success"] = False

        return result
    except Exception as e:
        return {"success": False, "error": str(e)}
