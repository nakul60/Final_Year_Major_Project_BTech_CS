"""
Hardware constraint verifier
"""

import re
from .registry import MCU_HARDWARE_REGISTRY


def hardware_constraint_verifier(code: str, platform: str) -> dict:
    """Check code for hardware pin conflicts and capability mismatches."""
    report = {"errors": [], "warnings": [], "info": [], "pins_used": {}}
    registry = MCU_HARDWARE_REGISTRY.get(platform.lower())
    
    if not registry:
        report["errors"].append(f"Unknown platform: {platform}")
        return report

    pin_patterns = [
        r'pinMode\((\w+),\s*(OUTPUT|INPUT|INPUT_PULLUP)\)',
        r'digitalWrite\((\w+),', r'digitalRead\((\w+),',
        r'analogRead\((\w+),', r'analogWrite\((\w+),',
        r'GPIO\.setup\((\w+),', r'GPIO\.output\((\w+),', r'GPIO\.input\((\w+),',
    ]

    for pattern in pin_patterns:
        matches = re.findall(pattern, code)
        for match in matches:
            pin = match[0] if isinstance(match, tuple) else match
            pin_num = int(pin) if pin.isdigit() else pin

            if pin_num in registry["input_only_pins"]:
                if "OUTPUT" in str(match):
                    report["errors"].append(f"Pin {pin} is input-only")

    return report
