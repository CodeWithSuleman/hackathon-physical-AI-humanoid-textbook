import os
import asyncio
from dotenv import load_dotenv
from agents import Agent, Runner, function_tool, AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig
from backend.src.retrieval.main import retrieve_chunks

load_dotenv()

openrouter_api_key = os.getenv("OPENROUTE_API_KEY")

external_client = AsyncOpenAI(
    api_key=openrouter_api_key,
    base_url="https://openrouter.ai/api/v1",
)

model = OpenAIChatCompletionsModel(
    model="mistralai/devstral-2512:free",
    openai_client=external_client,
)

config = RunConfig(
    model=model,
    model_provider=external_client,
    tracing_disabled=True,
)

@function_tool
def fetch_physical_ai_docs(query: str) -> str:
    """
    Physical AI database se information retrieve karne ke liye tool.
    Ye function Cohere embeddings aur Qdrant use karta hai.
    """
    chunks = retrieve_chunks(query, top_k=5)
    
    if not chunks:
        return "No relevant documents found."
        
    context = ""
    for c in chunks:
        context += f"\nContent: {c['content']}\nSource: {c['metadata'].get('source_url', 'N/A')}\n---"
    return context

rag_agent = Agent(
    name="PhysicalAIAgent",
    instructions="""
    You are an expert Physical AI assistant. 
    1. Always use 'fetch_physical_ai_docs' tool to find answers.
    2. Base your answer ONLY on the retrieved context.
    3. Cite the Source URL at the end of your response.
    """,
    tools=[fetch_physical_ai_docs],
)

async def chat_with_rag_agent(user_query: str):
    try:
        result = await Runner.run(
            rag_agent,
            input=user_query,
            run_config=config
        )
        
        return {
            "answer": str(result.final_output) if result.final_output else "I don't know.",
            "references": [] 
        }
    except Exception as e:
        print(f"Agent Error: {e}")
        return {"answer": f"Error: {str(e)}", "references": []}