import os
import uuid
import logging
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from urllib.parse import urljoin, urlparse

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


def get_all_urls(url, visited, base_domain):
    """
    Recursively crawls a website to get all unique URLs within the same domain.
    """
    if url in visited:
        return
    
    if urlparse(url).netloc != base_domain:
        return
        
    visited.add(url)
    try:
        response = requests.get(url, timeout=5)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, 'html.parser')
        
        for link in soup.find_all('a', href=True):
            absolute_link = urljoin(url, link['href']).split('#')[0]
            if absolute_link not in visited and urlparse(absolute_link).netloc == base_domain:
                get_all_urls(absolute_link, visited, base_domain)
    except requests.RequestException as e:
        logging.warning(f"Could not crawl {url}: {e}")
    except Exception as e:
        logging.error(f"An unexpected error occurred while crawling {url}: {e}")


def extract_text_from_url(url):
    """
    Extracts the main text content from a URL.
    """
    try:
        response = requests.get(url, timeout=5)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, 'html.parser')
        main_content = soup.find('main')
        if main_content:
            return main_content.get_text(separator=' ', strip=True)
        return ""
    except requests.RequestException as e:
        logging.warning(f"Could not fetch {url}: {e}")
        return ""
    except Exception as e:
        logging.error(f"An unexpected error occurred while extracting text from {url}: {e}")
        return ""


def chunk_text(text, chunk_size=384, overlap=48):
    """
    Splits a text into chunks of a specified size with a given overlap.
    """
    words = text.split()
    if not words:
        return []
    
    chunks = []
    for i in range(0, len(words), chunk_size - overlap):
        chunk = words[i:i + chunk_size]
        chunks.append(' '.join(chunk))
    return chunks


def embed_chunks(chunks, co_client):
    """
    Generates vector embeddings for a list of text chunks.
    """
    if not chunks:
        return []
    
    try:
        response = co_client.embed(texts=chunks, model="embed-english-v2.0", truncate="END")
        return response.embeddings
    except Exception as e:
        logging.error(f"An unexpected error occurred while embedding chunks: {e}")
        return []


def save_chunks_to_qdrant(qdrant, collection_name, chunks, embeddings, chunk_to_url_map):
    """
    Saves chunks and their embeddings to Qdrant.
    """
    if not embeddings:
        return
        
    points = []
    for i, embedding in enumerate(embeddings):
        points.append(PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={"text": chunks[i], "source_url": chunk_to_url_map[i]}
        ))

    qdrant.upsert(collection_name=collection_name, wait=True, points=points)


def main():
    """
    Main function to run the embedding pipeline.
    """
    load_dotenv()
    
    # Cohere client
    cohere_api_key = os.getenv("COHERE_API_KEY")
    co = cohere.Client(cohere_api_key)

    # Qdrant client
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    # Collection setup
    collection_name = "rag_embedding"
    
    # Forcefully delete and recreate with correct dimensions
    logging.info(f"Recreating collection '{collection_name}' with 4096 dims...")
    qdrant.recreate_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=4096, distance=Distance.COSINE),
    )
    logging.info(f"Collection '{collection_name}' is now FRESH and READY.")

    logging.info("Pipeline starting...")
    root_url = "https://codewithsuleman.github.io/hackathon-physical-AI-humanoid-textbook/"
    base_domain = urlparse(root_url).netloc
    visited_urls = set()
    logging.info(f"Crawling starting from {root_url}...")
    get_all_urls(root_url, visited_urls, base_domain)
    logging.info(f"Found {len(visited_urls)} URLs to process.")

    all_chunks = []
    chunk_to_url_map = []
    for url in visited_urls:
        logging.info(f"Extracting and chunking text from {url}...")
        text = extract_text_from_url(url)
        if text:
            chunks = chunk_text(text)
            all_chunks.extend(chunks)
            chunk_to_url_map.extend([url] * len(chunks))
    
    logging.info(f"Total chunks to process: {len(all_chunks)}")

    batch_size = 96
    for i in range(0, len(all_chunks), batch_size):
        batch_chunks = all_chunks[i:i + batch_size]
        batch_urls = chunk_to_url_map[i:i + batch_size]
        logging.info(f"Processing batch {i // batch_size + 1}...")
        
        batch_embeddings = embed_chunks(batch_chunks, co)
        if batch_embeddings:
            save_chunks_to_qdrant(qdrant, collection_name, batch_chunks, batch_embeddings, batch_urls)

    logging.info("Pipeline finished successfully!")


if __name__ == "__main__":
    main()
