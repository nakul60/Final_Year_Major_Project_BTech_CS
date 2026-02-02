"""
Web search tool for embedded systems information
"""

from langchain_core.tools import tool

try:
    from duckduckgo_search import DDGS
    WEB_SEARCH_AVAILABLE = True
except ImportError:
    WEB_SEARCH_AVAILABLE = False


@tool
def web_search_tool(query: str, max_results: int = 5) -> str:
    """Search the web for embedded systems information, tutorials, and documentation"""
    if not WEB_SEARCH_AVAILABLE:
        return "Web search not available. Please install duckduckgo-search package."

    try:
        ddgs = DDGS()
        results = []

        enhanced_query = f"{query} embedded systems arduino esp32 raspberry pi"
        search_results = ddgs.text(enhanced_query, max_results=max_results)

        for result in search_results:
            results.append({
                "title": result.get("title", ""),
                "url": result.get("href", ""),
                "snippet": result.get("body", "")
            })

        formatted_results = "🔍 Web Search Results:\n\n"
        for i, result in enumerate(results, 1):
            formatted_results += f"{i}. **{result['title']}**\n"
            formatted_results += f"   URL: {result['url']}\n"
            formatted_results += f"   {result['snippet'][:200]}...\n\n"

        return formatted_results

    except Exception as e:
        return f"Web search failed: {str(e)}"
