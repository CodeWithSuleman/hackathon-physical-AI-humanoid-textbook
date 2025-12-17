# Research: RAG Agent with OpenAI Agents SDK + FastAPI

This document outlines the research and decisions made for the RAG Agent with OpenAI Agents SDK + FastAPI feature.

## Decisions

### Retrieval Strategy

**Decision**: The RAG agent will primarily utilize similarity search (k-NN) through the existing retrieval module (Spec-2) to fetch relevant chunks from the Qdrant collection.

**Rationale**:
- The core requirement is to integrate the retrieval functionality with the agent. K-NN similarity search is the foundational method for semantic retrieval and directly supports this integration goal.
- The previous feature (Spec-2) already established the retrieval module based on k-NN.
- This approach keeps the initial implementation focused on the agent's ability to use the retrieval tool and synthesize information, without adding the complexity of advanced retrieval methods at this stage.

**Alternatives considered**:
- **Advanced Filtering**: While Qdrant supports complex metadata filtering, the initial focus is on semantic similarity. Complex filtering can be added in a future iteration if needed.
- **Hybrid Search (Vector + Keyword)**: Combining dense vector search with keyword-based search could improve recall for certain queries but is considered beyond the scope of this initial agent validation.
- **Re-ranking**: Applying a re-ranking model post-retrieval to improve the order of results was considered but is also outside the initial scope for simplicity.

### Prompt Template for Generation (OpenAI Agents SDK)

**Decision**: The OpenAI agent will be guided by detailed system instructions to ensure effective RAG. The prompt template will structure the agent's behavior as follows:

1.  **Instruction to Use Tools First**: Always prioritize the use of the defined retrieval tool (`knowledge_base_search` or similar) for factual questions.
2.  **Query Formulation Guidance**: Instruct the agent to formulate precise and comprehensive queries for the retrieval tool based on the user's question.
3.  **Information Synthesis**: Guide the agent to carefully read, synthesize, and summarize information from the retrieved chunks.
4.  **Reference Inclusion**: Explicitly require the agent to include references (URL + heading) from the source chunks in its final answer, as per the feature specification.
5.  **Graceful Handling of No Results**: Instruct the agent on how to respond if the retrieval tool returns no relevant information (e.g., politely inform the user, ask for clarification).
6.  **Avoid Hallucination**: Emphasize that the agent must not invent facts and should only respond with information directly supported by retrieved content or its general knowledge for non-factual queries.

**Rationale**:
- Effective system instructions are the cornerstone of reliable RAG with agents. They provide clear directives to the LLM on how to interact with tools and how to process and present information.
- By clearly defining these behaviors, the agent is more likely to generate coherent, well-supported answers with the required citations.

**Alternatives considered**:
- **Few-Shot Examples**: Providing a few examples of good RAG interactions within the prompt was considered. However, robust system instructions are often sufficient for core functionality, and few-shot examples can increase token usage. This can be explored in future iterations if the agent struggles with consistent output.
- **External Orchestration Logic**: Implementing complex external logic to pre-process retrieved chunks before passing them to the LLM (beyond basic concatenation). This is overly complex for the initial agent integration and the OpenAI Agents SDK is designed to handle this orchestration internally.
