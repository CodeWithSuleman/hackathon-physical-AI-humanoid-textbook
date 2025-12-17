from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import logging

from backend.src.rag_agent.agent import chat_with_rag_agent
from backend.src.rag_agent.models import QueryRequest, AgentResponse, Reference

logger = logging.getLogger(__name__)

app = FastAPI()

@app.post("/query", response_model=AgentResponse)
async def query_agent(request: QueryRequest):
    if not request.question:
        raise HTTPException(status_code=400, detail="Question cannot be empty.")
    
    try:
        response_data = chat_with_rag_agent(request.question)
        return AgentResponse(
            answer=response_data.get("answer", "No answer generated."),
            references=[Reference(**ref) for ref in response_data.get("references", [])]
        )
    except Exception as e:
        logger.error(f"Error processing query: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error while processing your request.")