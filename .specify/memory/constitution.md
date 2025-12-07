<!--
Sync Impact Report:
Version change: 0.0.0 → 1.0.0
List of modified principles:
  - PRINCIPLE_1_NAME → I. Hands-on Learning
  - PRINCIPLE_2_NAME → II. Accuracy
  - PRINCIPLE_3_NAME → III. Clarity
  - PRINCIPLE_4_NAME → IV. Integration
  - PRINCIPLE_5_NAME → V. Reproducibility
Added sections:
  - Key Standards
  - Constraints
  - Success Criteria
  - Stakeholders
  - Brand Voice
Removed sections:
  - None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs:
  - None
-->
# Physical AI & Humanoid Robotics Book with RAG Chatbot Constitution

## Core Principles

### I. Hands-on Learning
All content and examples MUST prioritize practical, hands-on learning experiences. The book's design and exercises must enable readers to actively engage with the material and apply concepts directly.

### II. Accuracy
All technical details, including those related to ROS 2, Gazebo, Unity, NVIDIA Isaac, and OpenAI, MUST be thoroughly verified against official documentation and peer-reviewed sources to ensure correctness.

### III. Clarity
Content MUST be written for a beginner-to-intermediate audience, using clear, concise language and providing necessary context and explanations for complex topics.

### IV. Integration
The RAG chatbot MUST strictly answer questions using only content derived from the book. Its responses MUST not introduce external information or make claims not supported by the book's text.

### V. Reproducibility
All code examples, simulations, and project setups provided within the book MUST be fully reproducible and runnable without errors by the target audience.

## Key Standards

- Correct, runnable code examples.
- References: Official documentation or peer-reviewed sources MUST be used and cited in APA style.
- Docusaurus Markdown with clear diagrams.
- Chatbot strictly uses book content.

## Constraints

- The Docusaurus book MUST be deployed to GitHub Pages.
- The book will comprise 4 modules plus a Capstone project, with a total word count between 10,000–15,000 words.
- The RAG system will utilize OpenAI Agents/ChatKit, FastAPI, Neon Postgres, and Qdrant Cloud.

## Success Criteria

- The book is live and accessible on GitHub Pages.
- All included code and simulations run without errors.
- The RAG chatbot accurately answers questions solely from the book's content.
- All citations are verified, and the content is free of plagiarism.

## Stakeholders

- Students, educators, robotics community.

## Brand Voice

- Friendly, educational, professional.

## Governance

This Constitution supersedes all other project practices. Amendments require thorough documentation, approval by stakeholders, and a clear migration plan for any affected components.

All pull requests and reviews MUST verify compliance with these principles. Complexity MUST be justified.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
