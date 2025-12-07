# scripts/ingest_data.py

import os
import asyncio
import asyncpg
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv
import tiktoken # For token-based chunking
from typing import List, Dict

# Assuming the backend/database.py is in the Python path or imported correctly
# from backend.database import get_connection, close_connection, init_db

load_dotenv()

# Environment variables for Neon Postgres
DATABASE_URL = os.getenv("DATABASE_URL")

# Environment variables for Qdrant Cloud
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = "ros2_book_chapters" # This can be made configurable

# Environment variables for OpenAI Embeddings (or other embedding model)
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
EMBEDDING_MODEL = "text-embedding-ada-002" # Or other model

# Initialize clients
qdrant_client = QdrantClient(
    host=QDRANT_HOST, 
    api_key=QDRANT_API_KEY,
)
# TODO: Initialize OpenAI client here

# --- Database functions (simplified from backend/database.py for direct use in script) ---
async def get_db_connection():
    if not DATABASE_URL:
        raise ValueError("DATABASE_URL environment variable is not set.")
    return await asyncpg.connect(DATABASE_URL)

async def store_content_in_neon(conn, chapter_id: str, section_title: str, content: str, chunk_id: str):
    """Stores a chunk of content in Neon Postgres."""
    await conn.execute(
        """
        INSERT INTO document_chunks (chapter_id, section_title, content, chunk_id)
        VALUES ($1, $2, $3, $4)
        ON CONFLICT (chunk_id) DO UPDATE SET
        chapter_id = EXCLUDED.chapter_id,
        section_title = EXCLUDED.section_title,
        content = EXCLUDED.content;
        """,
        chapter_id, section_title, content, chunk_id
    )

async def init_neon_schema(conn):
    """Initializes the Neon Postgres schema for document chunks."""
    await conn.execute("""
        CREATE TABLE IF NOT EXISTS document_chunks (
            id SERIAL PRIMARY KEY,
            chapter_id VARCHAR(255) NOT NULL,
            section_title TEXT,
            content TEXT NOT NULL,
            chunk_id VARCHAR(255) UNIQUE NOT NULL
        );
    """)
    print("Neon Postgres schema initialized.")

# --- Qdrant functions ---
async def create_qdrant_collection_if_not_exists():
    """Creates a Qdrant collection with specified vector size and distance."""
    # Assuming vector_size is 1536 for OpenAI ada-002 embeddings
    # This should ideally be read from a config or determined dynamically
    VECTOR_SIZE = 1536 
    if not qdrant_client.collection_exists(collection_name=QDRANT_COLLECTION_NAME):
        qdrant_client.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
        )
        print(f"Collection '{QDRANT_COLLECTION_NAME}' created in Qdrant.")
    else:
        print(f"Collection '{QDRANT_COLLECTION_NAME}' already exists in Qdrant.")

async def store_embeddings_in_qdrant(points: List[models.PointStruct]):
    """Stores a batch of vector embeddings and their payloads in Qdrant."""
    qdrant_client.upsert(
        collection_name=QDRANT_COLLECTION_NAME,
        wait=True,
        points=points,
    )
    print(f"Upserted {len(points)} points to Qdrant.")

# --- Markdown processing and chunking ---
def num_tokens_from_string(string: str, encoding_name: str = "cl100k_base") -> int:
    """Returns the number of tokens in a text string."""
    encoding = tiktoken.get_encoding(encoding_name)
    num_tokens = len(encoding.encode(string))
    return num_tokens

def chunk_markdown(file_path: str, max_tokens: int = 500, overlap_tokens: int = 50) -> List[Dict]:
    """
    Reads a markdown file, chunks its content based on token count,
    and returns a list of dictionaries, each representing a chunk.
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Basic chunking by paragraphs/lines first
    paragraphs = content.split('\n\n')
    chunks = []
    current_chunk_text = ""
    current_chapter_id = os.path.basename(file_path).replace(".md", "")
    current_section_title = "Introduction" # Default for chunks not explicitly in a section

    for para in paragraphs:
        if para.strip().startswith('#'): # Heuristic for section title
            current_section_title = para.strip().replace('#', '').strip()
            if num_tokens_from_string(current_chunk_text + "\n\n" + para) > max_tokens:
                 if current_chunk_text:
                     chunks.append({
                         "chapter_id": current_chapter_id,
                         "section_title": current_section_title, # Section title for the *next* chunk
                         "content": current_chunk_text.strip()
                     })
                     current_chunk_text = ""
            current_chunk_text += "\n\n" + para
            continue

        if num_tokens_from_string(current_chunk_text + "\n\n" + para) > max_tokens:
            if current_chunk_text:
                chunks.append({
                    "chapter_id": current_chapter_id,
                    "section_title": current_section_title,
                    "content": current_chunk_text.strip()
                })
            # Start new chunk with overlap
            current_chunk_text = " ".join(current_chunk_text.split()[-overlap_tokens:]) + "\n\n" + para
        else:
            current_chunk_text += "\n\n" + para
    
    if current_chunk_text:
        chunks.append({
            "chapter_id": current_chapter_id,
            "section_title": current_section_title,
            "content": current_chunk_text.strip()
        })

    # Generate unique chunk IDs
    for i, chunk in enumerate(chunks):
        chunk['chunk_id'] = f"{current_chapter_id}-{i}"

    return chunks

# --- Embedding generation (placeholder) ---
async def generate_embedding(text: str) -> List[float]:
    """
    Generates a vector embedding for the given text using an external model (e.g., OpenAI).
    TODO: Implement actual API call to OpenAI or other embedding service.
    """
    if not OPENAI_API_KEY:
        raise ValueError("OPENAI_API_KEY environment variable is not set for embedding generation.")
    
    # Placeholder for actual API call
    print(f"Generating embedding for text: '{text[:50]}...' ")
    # Simulate an embedding for now
    return [0.1] * 1536 # Return a dummy embedding of size 1536

# --- Main ingestion logic ---
async def ingest_documents(docs_path: str):
    """
    Ingests markdown documents from a specified path into Neon Postgres and Qdrant.
    """
    neon_conn = None
    try:
        neon_conn = await get_db_connection()
        await init_neon_schema(neon_conn)
        await create_qdrant_collection_if_not_exists()

        all_markdown_files = [os.path.join(docs_path, f) for f in os.listdir(docs_path) if f.endswith(".md")]
        
        for md_file in all_markdown_files:
            print(f"Processing file: {md_file}")
            chunks = chunk_markdown(md_file)
            
            qdrant_points = []
            for chunk in chunks:
                embedding = await generate_embedding(chunk["content"])
                
                # Store in Neon Postgres
                await store_content_in_neon(
                    neon_conn,
                    chapter_id=chunk["chapter_id"],
                    section_title=chunk["section_title"],
                    content=chunk["content"],
                    chunk_id=chunk["chunk_id"]
                )

                # Prepare for Qdrant
                qdrant_points.append(
                    models.PointStruct(
                        id=hash(chunk["chunk_id"]) % (2**63 - 1), # Simple hash for ID
                        vector=embedding,
                        payload={
                            "chapter_id": chunk["chapter_id"],
                            "section_title": chunk["section_title"],
                            "chunk_id": chunk["chunk_id"],
                            # Optionally store a snippet of content or reference to content
                            "content_snippet": chunk["content"][:200] + "..." 
                        }
                    )
                )
            
            if qdrant_points:
                await store_embeddings_in_qdrant(qdrant_points)
        
        print("Document ingestion complete.")

    except Exception as e:
        print(f"An error occurred during ingestion: {e}")
    finally:
        if neon_conn:
            await neon_conn.close()

if __name__ == "__main__":
    DOCS_MODULE_PATH = "docs/01-ros2-module-1" # Path to the markdown chapters
    asyncio.run(ingest_documents(DOCS_MODULE_PATH))
