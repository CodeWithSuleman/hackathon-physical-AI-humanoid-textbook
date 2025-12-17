# giaic-hackathonn Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-06

## Active Technologies
- Python 3.10+ (for Isaac Sim, Isaac ROS, Nav2 examples, and ROS 2), Markdown (for Docusaurus) + NVIDIA Isaac Sim, NVIDIA Isaac ROS, ROS 2 (Humble/Iron/Rolling), Nav2, Docusaurus (004-ai-robot-brain-module)
- N/A (Content and examples primarily stored in markdown and code files; no persistent database storage for this module's direct content. Synthetic data generation will output to local files/directories as part of demos.) (004-ai-robot-brain-module)
- Python 3.10+ (for Whisper, LLM integration, ROS 2 actions), Markdown (for Docusaurus) + OpenAI Whisper, a suitable LLM (e.g., OpenAI API, local LLM), ROS 2 (Humble/Iron/Rolling), Isaac Sim (for humanoid simulation), Isaac ROS (for perception/navigation primitives), Docusaurus (005-capstone-autonomous-humanoid)
- N/A (Content and examples primarily stored in markdown and code files; no persistent database storage for this module's direct content. Whisper audio/text, LLM prompts/responses, and ROS 2 action logs will be handled ephemerally within demos.) (005-capstone-autonomous-humanoid)
- Python 3.11 + `uv`, `cohere`, `qdrant-client`, `beautifulsoup4`, `requests`, `python-dotenv` (001-embedding-pipeline-setup)
- Qdrant Cloud, local `.env` file for credentials. (001-embedding-pipeline-setup)
- [e.g., Python 3.11, Swift 5.9, Rust 1.75 or NEEDS CLARIFICATION] + [e.g., FastAPI, UIKit, LLVM or NEEDS CLARIFICATION] (002-rag-retrieval-validation)
- [if applicable, e.g., PostgreSQL, CoreData, files or N/A] (002-rag-retrieval-validation)
- Python 3.11+ + `cohere`, `qdrant-client`, `uv` (002-rag-retrieval-validation)
- Python 3.11+ + `fastapi`, `openai`, `qdrant-client`, `cohere`, `uvicorn`, `python-dotenv` (003-rag-agent-fastapi)
- Qdrant Cloud (for document retrieval) (003-rag-agent-fastapi)

- Python 3.11 (for Gazebo/ROS 2 examples), C# (for Unity examples), Markdown (for Docusaurus) + Gazebo, Unity, ROS 2 (Humble), Docusaurus (003-digital-twin-module)

## Project Structure

```text
src/
tests/
```

## Commands

cd src; pytest; ruff check .

## Code Style

Python 3.11 (for Gazebo/ROS 2 examples), C# (for Unity examples), Markdown (for Docusaurus): Follow standard conventions

## Recent Changes
- 003-rag-agent-fastapi: Added Python 3.11+ + `fastapi`, `openai`, `qdrant-client`, `cohere`, `uvicorn`, `python-dotenv`
- 003-rag-agent-fastapi: Added [e.g., Python 3.11, Swift 5.9, Rust 1.75 or NEEDS CLARIFICATION] + [e.g., FastAPI, UIKit, LLVM or NEEDS CLARIFICATION]
- 002-rag-retrieval-validation: Added Python 3.11+ + `cohere`, `qdrant-client`, `uv`


<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->
