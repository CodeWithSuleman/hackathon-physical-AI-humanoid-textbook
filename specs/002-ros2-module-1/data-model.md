# Data Model for the Docusaurus Book

This document describes the data model for the book's content, which will be stored in a Neon Postgres database to be used by the RAG chatbot.

## Module

- **`id`**: Unique identifier for the module (e.g., `ros2-module-1`).
- **`title`**: The title of the module (e.g., "The Robotic Nervous System (ROS 2)").
- **`description`**: A brief description of the module's content.

## Chapter

- **`id`**: Unique identifier for the chapter (e.g., `01-ros2-basics`).
- **`module_id`**: Foreign key to the `Module` this chapter belongs to.
- **`title`**: The title of the chapter (e.g., "ROS 2 Basics").
- **`content`**: The full Markdown content of the chapter.

## Section

- **`id`**: Unique identifier for the section.
- **`chapter_id`**: Foreign key to the `Chapter` this section belongs to.
- **`title`**: The title of the section (e.g., "Nodes, Topics, and Services").
- **`content`**: The Markdown content of the section.

## Relationships

- A `Module` can have many `Chapters`.
- A `Chapter` belongs to one `Module`.
- A `Chapter` can have many `Sections`.
- A `Section` belongs to one `Chapter`.
