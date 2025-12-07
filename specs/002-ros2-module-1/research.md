# Research & Decisions

This document outlines the key decisions made during the planning phase for "Module 1: The Robotic Nervous System (ROS 2)".

## 1. RAG Pipeline Technology Stack

- **Decision**: The Retrieval-Augmented Generation (RAG) system for the chatbot will use Neon (Postgres) for storing the book's content and Qdrant Cloud for vector embeddings. The chatbot itself will be built with OpenAI's ChatKit and a FastAPI backend.
- **Rationale**: This stack was specified in the project's constitution. Neon provides a serverless Postgres experience that is easy to manage, Qdrant is a high-performance vector database suitable for semantic search, and FastAPI is a modern, fast framework for building the API to connect the components.
- **Alternatives considered**: Other vector databases like Pinecone or Weaviate were considered, but Qdrant was chosen for its performance and flexible deployment options. Other backend frameworks like Flask or Django could have been used, but FastAPI's performance and automatic documentation generation are well-suited for this project.

## 2. Depth of Technical Detail

- **Decision**: The technical depth will be targeted at a beginner-to-intermediate audience. Core concepts will be explained simply, with links to official documentation for more advanced topics. The focus will be on providing a solid foundational understanding rather than exhaustive detail.
- **Rationale**: The feature specification identifies the target audience as "Beginner-to-intermediate students". Overly technical content would alienate this audience. The goal is to make the content accessible and build a strong base of knowledge.
- **Alternatives considered**: A more advanced, in-depth treatment was considered but rejected as it would not serve the primary audience and would violate the "Clarity" principle of the constitution.

## 3. Docusaurus Content Layout

- **Decision**: The Docusaurus site will be structured into modules, with each module containing several chapters. For this feature, the structure will be a single module directory `docs/01-ros2-module-1/` containing Markdown files for each chapter.
- **Rationale**: This structure is simple, scalable, and aligns with the Docusaurus documentation's best practices. It allows for a clear and organized presentation of the content.
- **Alternatives considered**: A flat file structure was considered but rejected as it would become unmanageable as more modules are added.

## 4. Code Snippet Formatting

- **Decision**: All code snippets will be presented in standard Markdown code blocks, with the appropriate language specifier (e.g., `python`, `xml`, `bash`).
- **Rationale**: This ensures proper syntax highlighting and is the standard, most accessible way to present code in Markdown.
- **Alternatives considered**: Embedding code via gists or other services was considered but rejected to keep the content self-contained and easy to manage within the repository.

## 5. Citation Style

- **Decision**: All references to external documentation and sources will use the APA (American Psychological Association) citation style.
- **Rationale**: The constitution requires citations for accuracy. APA is a widely recognized standard in academic and technical writing.
- **Alternatives considered**: Other styles like MLA or Chicago were considered, but APA is a common choice for technical publications.
