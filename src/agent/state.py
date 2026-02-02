"""
State management for embedded systems agent
"""

from typing import Dict, List, Optional, Annotated
from typing_extensions import TypedDict
from langgraph.graph.message import add_messages


class ProjectState(TypedDict):
    """State management for embedded projects using TypedDict for LangGraph compatibility"""
    messages: Annotated[list, add_messages]
    platform: str
    requirements: str
    generated_code: str
    validation_result: Optional[Dict]
    documentation: str
    project_name: str
    current_step: str
    context_chunks: Optional[List[str]]
    error_message: str
    search_results: Optional[List[Dict]]
    iteration_count: int  # Track iterations to prevent infinite loops
