# Quickstart: Embedding Pipeline

**Branch**: `001-embedding-pipeline-setup` | **Date**: 2025-12-15 | **Plan**: [plan.md](plan.md)

This guide provides the steps to set up and run the embedding pipeline script.

## Prerequisites

- Python 3.10+ installed.
- `uv` installed (`pip install uv`).
- Access to Cohere and Qdrant Cloud accounts to get API keys.

## 1. Set up the Environment

First, navigate to the backend directory and set up the Python virtual environment using `uv`.

```bash
# Navigate to the script's directory
cd backend

# Create and activate a virtual environment
uv venv

# On Windows (PowerShell)
.venv\Scripts\Activate.ps1

# On macOS/Linux
source .venv/bin/activate
```

## 2. Install Dependencies

Install the required Python packages using `uv`.

```bash
uv pip install -r requirements.txt
```

## 3. Configure Credentials

Create a `.env` file in the `backend` directory. This file will hold the necessary API keys and URLs.

```bash
# backend/.env

COHERE_API_KEY="your_cohere_api_key_here"
QDRANT_URL="your_qdrant_cloud_url_here"
QDRANT_API_KEY="your_qdrant_api_key_here"
```

Replace the placeholder values with your actual credentials from Cohere and Qdrant.

## 4. Run the Pipeline

Execute the `main.py` script to start the embedding pipeline. The script will use the URL hardcoded in it from the user's request.

```bash
python main.py
```

The script will log its progress to the console, indicating that it is crawling URLs, extracting text, generating embeddings, and storing them in Qdrant. Upon completion, your Qdrant collection `rag_embedding` will be populated with the vectorized content from the website.
