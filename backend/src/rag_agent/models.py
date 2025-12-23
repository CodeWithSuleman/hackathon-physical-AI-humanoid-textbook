from pydantic import BaseModel
from typing import List, Optional
#.
class Reference(BaseModel):
    url: str
    heading: str

class QueryRequest(BaseModel):
    question: str

class AgentResponse(BaseModel):
    answer: str
    references: List[Reference]
