import os
from typing import List, Dict, Any
import cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Environment variables 
load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = "rag_embedding" 


co = cohere.Client(COHERE_API_KEY)

def get_embedding(text: str) -> List[float]:
    """converting text into vector through Cohere."""
    try:
        response = co.embed(
            texts=[text], 
            model="embed-english-v2.0", 
            truncate="END"
        )
        return response.embeddings[0]
    except Exception as e:
        print(f"❌ Cohere Error: {e}")
        return []

def retrieve_chunks(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """Retrieve relevant chunks from Qdrant."""
    if not query:
        return []

    # Initialize Qdrant client
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

    try:
        query_vector = get_embedding(query)
        if not query_vector:
            return []

        response = client.query_points(
            collection_name=QDRANT_COLLECTION_NAME,
            query=query_vector,
            limit=top_k,
            with_payload=True
        )

        results = []
        for point in response.points:
            results.append({
                "content": point.payload.get("text", "No content found"),
                "metadata": {
                    "source_url": point.payload.get("source_url", "No URL"),
                    "score": point.score
                }
            })
        
        print(f"{len(results)} chunks has recieved from Qdrant.")
        return results

    except Exception as e:
        print(f"Qdrant Retrieval Error: {e}")
        return []