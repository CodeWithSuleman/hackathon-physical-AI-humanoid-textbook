import os
import json
from openai import OpenAI
from dotenv import load_dotenv
from backend.src.retrieval.main import retrieve_chunks

load_dotenv()

# OpenAI Client setup for Gemini
client = OpenAI(
    api_key=os.getenv("GOOGLE_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

def chat_with_rag_agent(user_query: str):

    context_chunks = retrieve_chunks(user_query)
    context_text = "\n".join([f"SOURCE: {c['metadata']['source_url']}\nCONTENT: {c['content']}" for c in context_chunks])
    
    if not context_chunks:
        context_text = "No relevant context found in the textbook."

    messages = [
        {
            "role": "system", 
            "content": "You are a Physical AI expert. Use the provided context to answer questions. Cite sources clearly."
        },
        {
            "role": "user", 
            "content": f"Context:\n{context_text}\n\nQuestion: {user_query}"
        }
    ]

    try:
        response = client.chat.completions.create(
            model="gemini-1.5-flash", 
            messages=messages,
            temperature=0.1 
        )

        answer = response.choices[0].message.content
        
        # Format sources like the example code
        sources = list(set([c['metadata']['source_url'] for c in context_chunks]))

        return {
            "answer": answer,
            "references": [{"url": src, "heading": "Textbook Chapter"} for src in sources]
        }

    except Exception as e:
        error_msg = str(e)
        if "429" in error_msg:
            return {"answer": "⚠️ API Quota hit. Wait 30 seconds. Gemini 1.5-flash is busy.", "references": []}
        return {"answer": f"Error: {error_msg}", "references": []}