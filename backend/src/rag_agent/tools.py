from typing import List, Dict, Any
import json
import logging

from backend.src.retrieval.main import retrieve_chunks

logger = logging.getLogger(__name__)
#.
class RetrievalTool:
    """
    Tool to retrieve relevant chunks from the vector database.
    """
    def __init__(self, retrieval_function=retrieve_chunks):
        self.retrieval_function = retrieval_function

    def retrieve(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieves relevant text chunks from a vector database based on a natural language query.
        """
        logger.info(f"Retrieving chunks for query: '{query}' with top_k={top_k}")
        return self.retrieval_function(query, top_k)
