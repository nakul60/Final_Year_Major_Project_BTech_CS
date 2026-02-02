"""
Power consumption estimator for embedded systems
"""

import re
from typing import Dict
from langchain_core.tools import tool


MCU_PROFILES = {
    "esp32": {"active": 80, "wifi": 160, "bt": 90, "deep_sleep": 0.01},
    "arduino_uno": {"active": 45, "deep_sleep": 0.05},
    "arduino": {"active": 45, "deep_sleep": 0.05},
    "raspberry_pi_4": {"active": 600, "wifi": 200, "bt": 100, "deep_sleep": 10},
    "raspberry_pi": {"active": 600, "wifi": 200, "bt": 100, "deep_sleep": 10},
}

SENSORS = {"dht22": 1.5, "bme280": 0.7, "mpu6050": 3.9, "hc-sr04": 15, "led": 10}


def power_profile_estimator(code: str, platform: str) -> Dict:
    """Estimate power consumption for embedded code."""
    profile = MCU_PROFILES.get(platform.lower(), {"active": 100, "deep_sleep": 1})
    total_active = profile["active"]
    total_deep = profile.get("deep_sleep", 1)
    peripherals = []
    suggestions = []

    if "WiFi" in code or "wifi" in code:
        total_active += profile.get("wifi", 0)
        peripherals.append("WiFi")
    if "Bluetooth" in code or "SerialBT" in code:
        total_active += profile.get("bt", 0)
        peripherals.append("Bluetooth")
    for s in SENSORS:
        if s in code.lower():
            total_active += SENSORS[s]
            peripherals.append(s)

    if re.search(r'delay\s*\(', code):
        suggestions.append("Consider deep sleep instead of delay() for power savings")
        if "esp32" in platform.lower():
            suggestions.append("Use ESP.deepSleep() for ESP32")

    return {
        "active_mode_ma": round(total_active, 2),
        "deep_sleep_ma": round(total_deep, 2),
        "peripherals": peripherals,
        "suggestions": suggestions,
        "note": "Estimates are approximate"
    }


power_profile_estimator_fn = power_profile_estimator
power_profile_estimator_tool = tool(power_profile_estimator)
