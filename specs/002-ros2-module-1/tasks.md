# Tasks: Module 1: The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/002-ros2-module-1/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Docusaurus Project)

**Purpose**: Initialize the Docusaurus project and configure its basic structure.

- [ ] T001 Initialize a new Docusaurus site in the repository root.
- [ ] T002 Configure `docusaurus.config.js` with the book's title, theme, and navigation structure.
- [ ] T003 Create the initial directory structure for the book content under `docs/01-ros2-module-1/`.
- [ ] T004 [P] Customize the landing page in `src/pages/index.js`.

---

## Phase 2: Foundational (RAG Backend Prerequisites)

**Purpose**: Set up the cloud infrastructure for the RAG chatbot. This must be complete before the chatbot can be implemented.

- [x] T005 Set up a new project and database in Neon Postgres.
- [x] T006 Set up a new Qdrant Cloud instance and create a collection for vector embeddings.
- [ ] T007 Implement a data ingestion script (`scripts/ingest_data.py`) to read markdown files, chunk them, and store content in Neon and embeddings in Qdrant.

**Checkpoint**: RAG infrastructure is ready. Book content can be ingested.

---

## Phase 3: User Story 1 - Understand ROS 2 Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Create the first chapter of the book, explaining ROS 2 basics.

**Independent Test**: A user can read the chapter and successfully run the accompanying ROS 2 node example.

### Implementation for User Story 1

- [x] T008 [US1] Write the content for Chapter 1 in `docs/01-ros2-module-1/01-ros2-basics.md`, covering ROS 2 architecture, nodes, topics, and services.
- [x] T009 [US1] Create the example ROS 2 node (e.g., a simple publisher) in a new directory `examples/ros2_basics_node/`.
- [x] T010 [US1] Add instructions to the chapter on how to build and run the example node.

**Checkpoint**: The first chapter is complete and its example is runnable.

---

## Phase 4: User Story 2 - Control a Robot with Python (Priority: P2)

**Goal**: Create the second chapter, focusing on `rclpy`.

**Independent Test**: A user can read the chapter and run the Python script to publish a message to a ROS 2 topic.

### Implementation for User Story 2

- [x] T011 [US2] Write the content for Chapter 2 in `docs/01-ros2-module-1/02-python-agents.md`, explaining how to use `rclpy`.
- [x] T012 [US2] Create the example Python agent script (`examples/python_agent_publisher.py`) that publishes to a topic.
- [x] T013 [US2] Add instructions to the chapter on how to run the Python script.

**Checkpoint**: The second chapter is complete and its example is runnable.

---

## Phase 5: User Story 3 - Understand a Humanoid's Structure (Priority: P3)

**Goal**: Create the third chapter, explaining URDF.

**Independent Test**: A user can read the chapter and understand the structure of the provided URDF file.

### Implementation for User Story 3

- [x] T014 [US3] Write the content for Chapter 3 in `docs/01-ros2-module-1/03-urdf-for-humanoids.md`, explaining URDF syntax for links, joints, and sensors.
- [x] T015 [US3] Create a simple example URDF file (`examples/simple_humanoid.urdf`).
- [x] T016 [US3] Add instructions to the chapter on how to inspect the URDF file.

**Checkpoint**: The third chapter is complete and the example URdf is available.

---

## Phase 6: RAG Chatbot Implementation

**Purpose**: Build the backend service for the RAG chatbot.

- [ ] T017 Create a new FastAPI project for the chatbot backend in `backend/`.
- [ ] T018 Implement the `/chat` endpoint in `backend/main.py` according to `contracts/api.yaml`.
- [ ] T019 Implement the logic to query Qdrant for relevant document chunks based on the user's query.
- [ ] T020 Implement the logic to retrieve the full content of the relevant chunks from Neon Postgres.
- [ ] T021 Implement the logic to send the retrieved content and the user's query to an OpenAI model for answer generation.
- [ ] T022 [P] Add unit tests for the backend logic in `backend/tests/`.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final integration, testing, and deployment.

- [ ] T023 Integrate the RAG chatbot with the Docusaurus site, likely by creating a custom React component in `src/components/Chatbot/`.
- [ ] T024 Perform end-to-end testing of the book content, all examples, and the chatbot.
- [ ] T025 Verify that the Docusaurus site builds correctly with `npm run build`.
- [ ] T026 Deploy the site to GitHub Pages using the `docusaurus deploy` command or a custom GitHub Actions workflow.

---

## Dependencies & Execution Order

- **Setup (Phase 1)** must be completed first.
- **Foundational (Phase 2)** depends on Setup.
- **User Stories (Phases 3, 4, 5)** can be worked on in parallel after the Docusaurus structure is set up in Phase 1.
- **RAG Chatbot Implementation (Phase 6)** depends on the Foundational phase being complete.
- **Polish (Phase 7)** depends on all other phases being complete.
