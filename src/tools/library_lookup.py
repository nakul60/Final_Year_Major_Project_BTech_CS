"""
Library lookup tool
"""

from typing import Dict
from langchain_core.tools import tool


LIBRARIES = {
    "arduino": {
        "dht": {"name": "DHT sensor library", "install": "Arduino Library Manager", "example": "#include <DHT.h>"},
        "servo": {"name": "Servo", "install": "Built-in", "example": "#include <Servo.h>"},
        "wifi": {"name": "WiFi", "install": "Built-in for ESP32", "example": "#include <WiFi.h>"},
    },
    "raspberry_pi": {
        "rpi.gpio": {"name": "RPi.GPIO", "install": "pip install RPi.GPIO", "example": "import RPi.GPIO as GPIO"},
        "gpiozero": {"name": "GPIO Zero", "install": "pip install gpiozero", "example": "from gpiozero import LED"},
        "picamera": {"name": "PiCamera", "install": "pip install picamera", "example": "from picamera import PiCamera"},
    }
}


@tool
def library_lookup_tool(library_name: str, platform: str) -> Dict:
    """Look up library information and installation instructions"""
    platform_libs = LIBRARIES.get(platform.lower(), {})
    lib_key = library_name.lower().replace("-", "").replace("_", "").replace(".", "")

    for key, info in platform_libs.items():
        if lib_key in key or library_name.lower() in info["name"].lower():
            return {"success": True, "library": info}

    return {"error": f"Library '{library_name}' not found for '{platform}'"}
