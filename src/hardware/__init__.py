"""
Hardware module - MCU registry and verification
"""

from .registry import MCU_HARDWARE_REGISTRY
from .verifier import hardware_constraint_verifier

__all__ = ["MCU_HARDWARE_REGISTRY", "hardware_constraint_verifier"]
