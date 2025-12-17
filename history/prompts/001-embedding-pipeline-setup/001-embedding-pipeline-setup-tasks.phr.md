# Tasks: Embedding Pipeline Script

**Input**: Design documents from `specs/001-embedding-pipeline-setup/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1)

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create the basic file structure and configuration for the script.

- [x] T001 Create the `backend` directory for the script and its files.
- [x] T002 [P] Create `backend/requirements.txt` and add all necessary dependencies: `cohere`, `qdrant-client`, `beautifulsoup4`, `requests`, `python-dotenv`.
- [x] T003 [P] Create `backend/.env.example` with placeholder values for `COHERE_API_KEY`, `QDRANT_URL`, and `QDRANT_API_KEY`.
- [x] T004 Create the main script file `backend/main.py` and add basic imports and a placeholder `main()` function.

---

## Phase 2: Foundational (Configuration & Clients)

**Purpose**: Implement the core setup for connecting to external services.

**⚠️ CRITICAL**: No pipeline logic can be built until this is complete.

- [x] T005 In `backend/main.py`, add the code to load environment variables from the `.env` file.
- [x] T006 In `backend/main.py`, implement the initialization of the Cohere client using the loaded API key.
- [x] T007 In `backend/main.py`, implement the initialization of the Qdrant client using the loaded URL and API key.
- [x] T008 In `backend/main.py`, add logic to connect to Qdrant and create the `rag_embedding` collection if it does not already exist.

**Checkpoint**: Foundation ready. The script can connect to Cohere and Qdrant.

---

## Phase 3: User Story 1 - RAG Ingestion Pipeline 🎯 MVP

**Goal**: Implement the core logic to crawl a website, extract its content, and store it as vector embeddings in Qdrant.

**Independent Test**: Run `python backend/main.py`. The script should execute without errors, and the Qdrant `rag_embedding` collection should contain new data points corresponding to the content of the target URL.

### Implementation for User Story 1

- [x] T009 [US1] Implement the `get_all_urls` function in `backend/main.py`. This function should take a root URL and recursively find all unique URLs within the same domain.
- [x] T010 [US1] Implement the `extract_text_from_url` function in `backend/main.py`. This function should use `requests` and `BeautifulSoup` to fetch and parse the HTML, extracting only the main textual content.
- [x] T011 [US1] Implement the `chunk_text` function in `backend/main.py`. This function will take a string of text and split it into chunks of 512 tokens with a 64-token overlap.
- [x] T012 [US1] Implement the `embed_chunks` function in `backend/main.py`. This function will call the Cohere client to generate vector embeddings for a list of text chunks.
- [x] T013 [US1] Implement the `save_chunks_to_qdrant` function in `backend/main.py`. This function will take the text chunks and their embeddings and upsert them as points into the `rag_embedding` collection in Qdrant.
- [x] T014 [US1] Implement the main execution logic within the `main()` function in `backend/main.py`. This logic will orchestrate the entire pipeline by calling the functions created in T009-T013 in the correct order.

**Checkpoint**: At this point, User Story 1 should be fully functional and meet the MVP requirements.

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Improve the script's robustness, usability, and documentation.

- [x] T015 Add structured logging (e.g.,INFO, WARNING, ERROR) to all major functions within `backend/main.py` to provide visibility into the pipeline's execution.
- [ ] T016 Implement robust error handling (e.g., using try-except blocks) for network requests, API calls, and file operations in `backend/main.py`.
- [ ] T017 [P] Create a `backend/README.md` file that clearly explains what the script does, how to set it up (`requirements.txt`, `.env`), and how to run it.
- [ ] T018 Perform a final code review and refactoring of `backend/main.py` for clarity, style, and efficiency.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion.
- **User Story 1 (Phase 3)**: Depends on Foundational phase completion.
- **Polish (Phase 4)**: Depends on User Story 1 completion.

### Parallel Opportunities

- Within Phase 1, T002 and T003 can be done in parallel.
- Since this is a single-file script, most implementation tasks (T009-T014) are sequential as they will be in the same file. However, the logic for each function can be developed independently before being integrated.
- T017 (documentation) can be worked on in parallel with other polish tasks.

---

## Implementation Strategy

### MVP First (User Story 1)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete all tasks in Phase 3: User Story 1.
4.  **STOP and VALIDATE**: Test the script by running it and checking the output in Qdrant. At this point, the core feature is delivered.
5.  Proceed to Phase 4 for improvements.
