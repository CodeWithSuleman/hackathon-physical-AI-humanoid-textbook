# Data Model: RAG Retrieval & Pipeline Validation

This document describes the key data entities for the RAG Retrieval & Pipeline Validation feature.

## Query

Represents a natural language query for information retrieval.

**Attributes**:
- `text`: (string) The text of the query.

## Text Chunk

Represents a piece of text retrieved from the vector database.

**Attributes**:
- `content`: (string) The text of the chunk.
- `metadata`: (dictionary) A collection of key-value pairs associated with the chunk (e.g., source, page number).
- `score`: (float) The similarity score of the chunk with respect to the query.
