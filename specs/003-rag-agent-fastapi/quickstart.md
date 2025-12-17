# Quickstart: RAG Agent with OpenAI Agents SDK + FastAPI

This document provides a quickstart guide for setting up and interacting with the RAG Agent FastAPI application.

## Prerequisites

- Python 3.11+
- `uv` for dependency management
- An OpenAI API Key (for Agents SDK)
- An existing Qdrant collection populated with embeddings (from Spec-1 and Spec-2)
- Qdrant URL and API Key

## Installation

The RAG Agent will be implemented within the `backend` project.

1.  Navigate to the `backend` directory:
    ```bash
    cd backend
    ```
2.  Install the required dependencies:
    ```bash
    uv pip install "fastapi[all]" openai qdrant-client python-dotenv cohere uvicorn
    ```
    (Note: `cohere` and `qdrant-client` might already be installed from previous specs)

## Configuration

Create a `.env` file in the `backend` directory with the following environment variables:

```
OPENAI_API_KEY="your_openai_api_key_here"
QDRANT_URL="your_qdrant_instance_url"
QDRANT_API_KEY="your_qdrant_api_key"
QDRANT_COLLECTION_NAME="your_qdrant_collection_name"
COHERE_API_KEY="your_cohere_api_key"
```

## Running the Application

1.  Make sure you are in the `backend` directory.
2.  Run the FastAPI application:
    ```bash
    uvicorn main:app --reload
    ```
    (Note: The main application file might be named differently, e.g., `agent_app:app` or similar, depending on the implementation.)

## Interacting with the API

Once the application is running, you can interact with the `/query` endpoint.

### Example using `curl`

```bash
curl -X POST "http://127.0.0.1:8000/query" \
     -H "Content-Type: application/json" \
     -d '{
       "question": "What is the physical AI humanoid robotics textbook about?"
     }'
```

### Example using Python `requests`

```python
import requests
import json

url = "http://127.00.1:8000/query"
headers = {"Content-Type": "application/json"}
data = {"question": "What are the core principles of physical AI?"}

response = requests.post(url, headers=headers, data=json.dumps(data))

if response.status_code == 200:
    print("Success:")
    print(json.dumps(response.json(), indent=2))
else:
    print(f"Error: {response.status_code}")
    print(response.text)
```

