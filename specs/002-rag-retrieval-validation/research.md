# Research: RAG Retrieval & Pipeline Validation

This document outlines the research and decisions made for the RAG Retrieval & Pipeline Validation feature.

## Decisions

### `top-k` Value

**Decision**: The retrieval module will use a default `top-k` value of 5.

**Rationale**: 
- A `top-k` of 5 is a good starting point for providing focused and relevant results without overwhelming the user.
- This value aligns with the success criterion `SC-001`, which specifies measuring `precision@5`.
- The research indicates that starting with a smaller `k` (e.g., 3-5) is a best practice for applications requiring precise answers.

**Alternatives considered**:
- A higher `top-k` value (e.g., 10-15) was considered but rejected to avoid introducing noise and to keep the initial implementation simple.
- A dynamic `top-k` value was considered out of scope for this initial implementation.

### Score Threshold

**Decision**: The retrieval module will not use a fixed score threshold to filter results. Instead, it will return the `top-k` results along with their similarity scores.

**Rationale**:
- There is no universal "best" score threshold. It depends heavily on the embedding model and the dataset.
- Returning the scores provides more flexibility to the consumer of the module, who can then decide how to interpret the scores.
- This approach avoids making assumptions about the relevance of a particular score.

**Alternatives considered**:
- A hardcoded score threshold was rejected because it would be arbitrary and might not be optimal.
- An empirically determined threshold was considered out of scope for this initial validation feature.

### Response Format

**Decision**: The retrieval function will return a list of dictionaries. Each dictionary will represent a retrieved chunk and will contain the `content` of the chunk, its `metadata`, and the retrieval `score`.

**Example**:
```json
[
  {
    "content": "This is the content of the first chunk.",
    "metadata": { "source": "doc1.txt", "page": 1 },
    "score": 0.95
  },
  {
    "content": "This is the content of the second chunk.",
    "metadata": { "source": "doc2.txt", "page": 3 },
    "score": 0.92
  }
]
```

**Rationale**:
- This format is simple, easy to parse, and provides all the necessary information to the consumer.
- It is a common and best practice for returning search results from an API or module.

**Alternatives considered**:
- Returning a more complex object with pagination was considered but deemed unnecessary for this initial validation feature, which focuses on a single retrieval operation.
