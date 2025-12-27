from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from backend.src.rag_agent.agent import chat_with_rag_agent
from backend.src.rag_agent.models import QueryRequest, AgentResponse

app = FastAPI()

# --- NEW: CORS Configuration ---
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://codewithsuleman.github.io"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
@app.get("/")
def health():
    return {"status": "ok"}
#.
@app.post("/query", response_model=AgentResponse)
async def query_endpoint(request: QueryRequest):
    if not request.question:
        raise HTTPException(status_code=400, detail="Empty question")
    
    try:
        response_data = await chat_with_rag_agent(request.question)
        return AgentResponse(
            answer=response_data["answer"],
            references=[]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))