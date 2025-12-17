# Quickstart: RAG Retrieval Module

This document provides a quickstart guide for using the RAG retrieval module.

## Installation

The retrieval module is part of the `backend` project. To use it, you first need to install the dependencies of the project:

```bash
cd backend
uv pip install -r requirements.txt
```

## Usage

The retrieval module provides a simple function to retrieve relevant text chunks from the vector database.

```python
from retrieval import retrieve_chunks

query = "What is the capital of France?"
top_k = 5

results = retrieve_chunks(query, top_k)

for result in results:
    print(f"Content: {result['content']}")
    print(f"Metadata: {result['metadata']}")
    print(f"Score: {result['score']}")
    print("-" * 20)
```

### `retrieve_chunks(query, top_k)`

**Parameters**:
- `query` (str): The natural language query.
- `top_k` (int): The number of top results to retrieve.

**Returns**:
- A list of dictionaries, where each dictionary represents a retrieved chunk and contains the `content`, `metadata`, and `score`.
