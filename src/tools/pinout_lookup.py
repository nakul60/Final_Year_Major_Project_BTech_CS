"""
Pinout lookup tool for microcontrollers
"""

from langchain_core.tools import tool


PINOUTS = {
    "arduino_uno": {
        "description": "Arduino Uno R3 Pinout",
        "digital_pins": "0-13 (pins 0,1 are RX,TX for serial)",
        "analog_pins": "A0-A5 (can also be digital pins 14-19)",
        "power_pins": "3.3V, 5V, GND, VIN (7-12V input)",
        "pwm_pins": "3, 5, 6, 9, 10, 11 (marked with ~)",
        "special_pins": {
            "I2C SDA": "A4", "I2C SCL": "A5",
            "SPI SS": "10", "SPI MOSI": "11", "SPI MISO": "12", "SPI SCK": "13",
            "LED_BUILTIN": "13"
        },
        "notes": "- Pins 0,1 for USB\n- Pin 13 has built-in LED\n- Max 20mA per pin"
    },
    "esp32": {
        "description": "ESP32 Development Board Pinout",
        "digital_pins": "0-39 (input-only: 34, 35, 36, 39)",
        "analog_pins": "32-39, 25-27, 12-15, 2, 0, 4 (12-bit ADC)",
        "touch_pins": "0, 2, 4, 12, 13, 14, 15, 27, 32, 33",
        "pwm_pins": "All digital pins support PWM",
        "special_pins": {
            "I2C SDA": "21", "I2C SCL": "22",
            "UART RX": "3", "UART TX": "1",
            "SPI SS": "5", "SPI MOSI": "23", "SPI MISO": "19", "SPI SCK": "18",
            "Built-in LED": "2"
        },
        "notes": "- WiFi/BT built-in\n- 3.3V logic\n- Pins 6-11 for flash"
    },
    "raspberry_pi": {
        "description": "Raspberry Pi 4 GPIO Pinout (40-pin)",
        "gpio_pins": "GPIO 2-27",
        "power_pins": "3.3V (1,17), 5V (2,4), GND (6,9,14,20,25,30,34,39)",
        "special_pins": {
            "I2C SDA": "GPIO 2 (pin 3)", "I2C SCL": "GPIO 3 (pin 5)",
            "UART RX": "GPIO 15 (pin 10)", "UART TX": "GPIO 14 (pin 8)",
            "SPI MOSI": "GPIO 10", "SPI MISO": "GPIO 9", "SPI SCK": "GPIO 11",
            "PWM0": "GPIO 12", "PWM1": "GPIO 13"
        },
        "notes": "- 3.3V logic (NOT 5V tolerant!)\n- Max 16mA per pin"
    }
}


@tool
def pinout_lookup_tool(platform: str) -> str:
    """Get detailed pinout information for microcontrollers and development boards"""
    platform_key = platform.lower().replace("-", "_").replace(" ", "_")

    if platform_key in PINOUTS:
        info = PINOUTS[platform_key]
        result = f"📌 **{info['description']}**\n\n"

        for key in ['digital_pins', 'analog_pins', 'gpio_pins', 'power_pins', 'pwm_pins', 'touch_pins']:
            if key in info:
                result += f"**{key.replace('_', ' ').title()}:** {info[key]}\n"

        result += f"\n**Special Functions:**\n"
        for func, pin in info['special_pins'].items():
            result += f"- {func}: {pin}\n"

        result += f"\n**Notes:**\n{info['notes']}"
        return result
    
    return f"❌ Pinout for '{platform}' not found. Available: {', '.join(PINOUTS.keys())}"
