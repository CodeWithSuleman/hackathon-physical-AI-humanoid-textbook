import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
from backend.src.main import app # Assuming main.py is the entry point for the FastAPI app
from qdrant_client.http.models import ScoredPoint
# from openai.types.beta.threads import MessageDeltaEvent # COMMENTED OUT
# from openai.lib.streaming import AssistantEventHandler # COMMENTED OUT
# from openai.types.beta.threads.runs import ToolCall # COMMENTED OUT
# from openai.types.beta.threads.messages import Message # COMMENTED OUT
import json

client = TestClient(app)

@pytest.fixture(autouse=True)
def mock_env_vars(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "test_openai_key")
    monkeypatch.setenv("COHERE_API_KEY", "test_cohere_key")
    monkeypatch.setenv("QDRANT_URL", "http://localhost:6333")
    monkeypatch.setenv("QDRANT_API_KEY", "test_qdrant_key")
    monkeypatch.setenv("QDRANT_COLLECTION_NAME", "test_collection")

@patch('backend.src.rag_agent.agent.chat_with_rag_agent')
def test_empty_question_query_endpoint(mock_chat_with_rag_agent):
    """
    Test case for T009: empty question to `/query` endpoint (expect 400).
    """
    response = client.post("/query", json={"question": ""})
    assert response.status_code == 400
    assert response.json() == {"detail": "Question cannot be empty."}

@patch('backend.src.rag_agent.agent.chat_with_rag_agent')
def test_valid_question_coherent_answer_with_references(mock_chat_with_rag_agent):
    """
    Test case for T010: valid question returning coherent answer with references.
    """
    mock_chat_with_rag_agent.return_value = {
        "answer": "Physics is a natural science. Source: [What is Physics?](https://example.com/physics)",
        "references": [
            {"url": "https://example.com/physics", "heading": "What is Physics?"},
            {"url": "https://example.com/quantum", "heading": "Quantum Mechanics"}
        ]
    }

    response = client.post("/query", json={"question": "What is physics?"})
    assert response.status_code == 200
    response_json = response.json()
    assert response_json["answer"] == "Physics is a natural science. Source: [What is Physics?](https://example.com/physics)"
    assert len(response_json["references"]) == 2
    assert response_json["references"][0]["url"] == "https://example.com/physics"
    assert response_json["references"][0]["heading"] == "What is Physics?"
    assert response_json["references"][1]["url"] == "https://example.com/quantum"
    assert response_json["references"][1]["heading"] == "Quantum Mechanics"

@patch('backend.src.rag_agent.agent.chat_with_rag_agent')
def test_question_unrelated_to_indexed_documents(mock_chat_with_rag_agent):
    """
    Test case for T011: question unrelated to indexed documents.
    If retrieval yields no relevant chunks, the agent should still provide a graceful response.
    """
    mock_chat_with_rag_agent.return_value = {
        "answer": "I'm sorry, I cannot find relevant information in my knowledge base for that question.",
        "references": []
    }

    response = client.post("/query", json={"question": "Tell me a joke."})
    assert response.status_code == 200
    response_json = response.json()
    assert "I'm sorry" in response_json["answer"]
    assert len(response_json["references"]) == 0
