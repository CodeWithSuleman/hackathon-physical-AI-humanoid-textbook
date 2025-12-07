# Research & Decisions: Capstone Project: Full Autonomous Humanoid (ROS 2/Isaac)

This document outlines key decisions made during the planning phase for the "Capstone Project: Full Autonomous Humanoid (ROS 2/Isaac)".

## 1. Handling Ephemeral Data Storage for Demos

-   **Decision**: Demos will handle data such as Whisper audio/text, LLM prompts/responses, and ROS 2 action logs ephemerally. This means data will be processed and used during the demonstration but will not be persistently stored in a database or complex file system for the purpose of the module's content.
-   **Rationale**: The focus of this module is on demonstrating the VLA pipeline and autonomous task execution, not on data persistence or management. Handling data ephemerally simplifies the examples, reduces setup complexity for students, and keeps the focus on the core concepts.
-   **Alternatives considered**: Implementing persistent storage for demo data was considered but deemed unnecessary and would add complexity that detracts from the learning objectives for this beginner-to-intermediate audience. Outputs will primarily be displayed in console or temporary files for immediate observation.
