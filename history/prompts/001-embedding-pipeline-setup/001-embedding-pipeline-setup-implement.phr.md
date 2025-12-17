This file summarizes the implementation of the embedding pipeline.

The implementation involved:
- Setting up the Python environment with `uv`.
- Installing dependencies: `cohere`, `qdrant-client`, `beautifulsoup4`, `requests`, `python-dotenv`.
- Creating a script `scripts/ingest_data.py` to:
    - Scrape text data from specified URLs.
    - Generate embeddings using the Cohere API.
    - Ingest the embeddings into a Qdrant Cloud collection.
- The implementation is configurable via a `.env` file.
