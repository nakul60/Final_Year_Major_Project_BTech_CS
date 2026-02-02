"""
Tools module - All LangGraph tools for embedded systems
"""

from .web_search import web_search_tool
from .component_lookup import component_lookup_tool
from .pinout_lookup import pinout_lookup_tool
from .code_templates import code_template_tool
from .code_validator import code_validator_tool
from .library_lookup import library_lookup_tool
from .file_operations import file_operations_tool
from .power_estimator import power_profile_estimator_tool, power_profile_estimator_fn


def get_all_tools():
    """Return all available tools for the agent"""
    return [
        web_search_tool,
        component_lookup_tool,
        pinout_lookup_tool,
        code_template_tool,
        code_validator_tool,
        library_lookup_tool,
        file_operations_tool,
    ]


__all__ = [
    "get_all_tools",
    "web_search_tool",
    "component_lookup_tool", 
    "pinout_lookup_tool",
    "code_template_tool",
    "code_validator_tool",
    "library_lookup_tool",
    "file_operations_tool",
    "power_profile_estimator_tool",
    "power_profile_estimator_fn",
]
