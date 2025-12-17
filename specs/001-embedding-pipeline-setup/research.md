# Research & Decisions: Embedding Pipeline

**Branch**: `001-embedding-pipeline-setup` | **Date**: 2025-12-15 | **Plan**: [plan.md](plan.md)

This document records the decisions made to resolve points of clarification from the feature specification.

## 1. Text Chunking Strategy

- **Decision**: Use a fixed-size chunking strategy. Text will be split into chunks of **512 tokens** with an overlap of **64 tokens**.
- **Rationale**: This approach was chosen for its simplicity and effectiveness as a baseline. It is straightforward to implement and provides a good balance between context preservation (through overlap) and manageable chunk sizes for the embedding model. This decision aligns with the user's request for a simple, single-file script.
- **Alternatives Considered**:
  - *Recursive Character Splitting*: While potentially yielding better semantic chunks, it adds complexity that contradicts the goal of a simple, single-file implementation.
  - *Configurable Strategy*: Making the strategy user-configurable would add significant complexity to the script's interface and logic.

## 2. Pipeline Trigger Mechanism

- **Decision**: The pipeline will be triggered via a **manual command-line script**.
- **Rationale**: The user explicitly requested the entire logic be contained in a single `main.py` file to be executed. A manual CLI trigger is the most direct interpretation of this requirement.
- **Alternatives Considered**:
  - *API Endpoint*: This would require setting up a web server (e.g., FastAPI), which goes against the single-script architecture.
  - *Scheduled Job*: This would add external dependencies (like `cron` or a cloud scheduler) and complexity beyond the scope of a simple script.

## 3. Credentials Management

- **Decision**: Credentials and API keys for Cohere and Qdrant will be managed using a **`.env` file**.
- **Rationale**: This is a standard, secure practice for Python applications. It keeps sensitive information out of version control and separates configuration from the code, while still being simple enough for a single-script project. The `python-dotenv` library will be used to load these variables.
- **Alternatives Considered**:
  - *Environment Variables*: A valid option, but a `.env` file is more portable and easier for users to manage without polluting their shell environment.
  - *Cloud Secrets Manager*: Overkill for a simple script and would introduce unnecessary complexity and cloud provider dependencies.
