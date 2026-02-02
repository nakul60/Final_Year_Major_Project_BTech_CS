"""
MCU Hardware Registry - Hardware specifications for supported platforms
"""

MCU_HARDWARE_REGISTRY = {
    "esp32": {
        "input_only_pins": [34, 35, 36, 39],
        "analog_input_pins": [0, 2, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, 39],
        "pwm_capable_pins": list(range(0, 34)),
        "i2c_default_pins": {"sda": 21, "scl": 22},
        "max_io_voltage": 3.3
    },
    "arduino_uno": {
        "input_only_pins": [],
        "analog_input_pins": ["A0", "A1", "A2", "A3", "A4", "A5"],
        "pwm_capable_pins": [3, 5, 6, 9, 10, 11],
        "i2c_default_pins": {"sda": "A4", "scl": "A5"},
        "max_io_voltage": 5.0
    },
    "raspberry_pi_4": {
        "input_only_pins": [],
        "analog_input_pins": [],
        "pwm_capable_pins": [12, 13, 18, 19],
        "i2c_default_pins": {"sda": 2, "scl": 3},
        "max_io_voltage": 3.3
    }
}
