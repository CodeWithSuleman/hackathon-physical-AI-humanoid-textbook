---

description: "Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts."
---

# Tasks: RAG Agent with OpenAI Agents SDK + FastAPI

**Input**: Design documents from `/specs/003-rag-agent-fastapi/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/query_api.yaml, quickstart.md

**Tests**: This feature explicitly requests end-to-end tests for the `/query` endpoint.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create `backend/src/rag_agent` directory `backend/src/rag_agent`
- [X] T002 Add `fastapi`, `openai`, `uvicorn`, `httpx` to `backend/requirements.txt` `backend/requirements.txt`
- [X] T003 Install dependencies using `uv` from `backend/requirements.txt`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Implement `__init__.py` for the `rag_agent` module `backend/src/rag_agent/__init__.py`
- [X] T005 Create `backend/src/rag_agent/tools.py` for OpenAI Agents SDK tools `backend/src/rag_agent/tools.py`
- [X] T006 [P] Implement `retrieve_chunks_tool` in `backend/src/rag_agent/tools.py` that wraps `retrieval.main.retrieve_chunks` `backend/src/rag_agent/tools.py`
- [X] T007 Define `QueryRequest` and `AgentResponse` Pydantic models in `backend/src/rag_agent/models.py` `backend/src/rag_agent/models.py`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Querying the RAG Agent (Priority: P1) 🎯 MVP

**Goal**: A user sends a natural language question to the backend agent via an API endpoint, and the agent responds with a coherent answer based on retrieved documents, including references to the source.

**Independent Test**: Can be fully tested by sending a POST request to the `/query` endpoint with a question and verifying the JSON response contains a coherent answer and correct source references.

### Tests for User Story 1

- [X] T008 [P] [US1] Create integration test file for RAG API `backend/tests/integration/test_rag_api.py`
- [X] T009 [P] [US1] Integration test for empty question to `/query` endpoint (expect 400) `backend/tests/integration/test_rag_api.py`
- [X] T010 [P] [US1] Integration test for valid question returning coherent answer with references `backend/tests/integration/test_rag_api.py`
- [X] T011 [P] [US1] Integration test for question unrelated to indexed documents `backend/tests/integration/test_rag_api.py`

### Implementation for User Story 1

- [X] T012 [P] [US1] Create `backend/src/rag_agent/agent.py` for core RAG agent logic `backend/src/rag_agent/agent.py`
- [X] T013 [P] [US1] Initialize OpenAI client in `backend/src/rag_agent/agent.py` `backend/src/rag_agent/agent.py`
- [X] T014 [P] [US1] Define system instructions for the OpenAI agent `backend/src/rag_agent/agent.py`
- [X] T015 [P] [US1] Initialize the OpenAI agent with the `retrieve_chunks_tool` `backend/src/rag_agent/agent.py`
- [X] T016 [P] [US1] Implement agent's response generation logic to include answer and references `backend/src/rag_agent/agent.py`
- [X] T017 [US1] Create FastAPI application instance `backend/src/main.py`
- [X] T018 [US1] Implement `/query` POST endpoint in `backend/src/main.py` `backend/src/main.py`
- [X] T019 [US1] Integrate RAG agent with `/query` endpoint to process questions and return responses `backend/src/main.py`
- [X] T020 [US1] Add error handling for OpenAI/Qdrant failures and empty questions `backend/src/main.py`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T021 Run `quickstart.md` validation to ensure the example works `specs/003-rag-agent-fastapi/quickstart.md`
- [X] T022 Code cleanup and refactoring for the `rag_agent` module `backend/src/rag_agent/`
- [ ] T023 Ensure new technologies (`fastapi`, `openai`, `uvicorn`, `httpx`) are documented in `GEMINI.md` and `CLAUDE.md` `GEMINI.md`, `CLAUDE.md`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Tests MUST be written and FAIL before implementation
- Models before services
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks T001, T002, T003 can be executed sequentially for environment setup.
- Foundational tasks T005, T006, T007 are parallelizable.
- Within User Story 1, tasks T008, T009, T010, T011 (tests) and T012, T013, T014, T015, T016 (agent implementation) are parallelizable.
- Tasks T017, T018, T019, T020 are sequential and integrate the agent with FastAPI.

---

## Parallel Example: User Story 1

```bash
# Core components for User Story 1 can be developed in parallel:
Task: "T005 [P] [US1] Implement `retrieve_chunks_tool` in `backend/src/rag_agent/tools.py`"
Task: "T006 [P] [US1] Define `QueryRequest` and `AgentResponse` Pydantic models in `backend/src/rag_agent/models.py`"
Task: "T012 [P] [US1] Create `backend/src/rag_agent/agent.py` for core RAG agent logic"
Task: "T013 [P] [US1] Initialize OpenAI client in `backend/src/rag_agent/agent.py`"
Task: "T014 [P] [US1] Define system instructions for the OpenAI agent `backend/src/rag_agent/agent.py`"
Task: "T015 [P] [US1] Initialize the OpenAI agent with the `retrieve_chunks_tool` `backend/src/rag_agent/agent.py`"
Task: "T016 [P] [US1] Implement agent's response generation logic to include answer and references `backend/src/rag_agent/agent.py`"

# Integration tests can also be developed in parallel:
Task: "T008 [P] [US1] Create integration test file for RAG API `backend/tests/integration/test_rag_api.py`"
Task: "T009 [P] [US1] Integration test for empty question to `/query` endpoint (expect 400) `backend/tests/integration/test_rag_api.py`"
Task: "T010 [P] [US1] Integration test for valid question returning coherent answer with references `backend/tests/integration/test_rag_api.py`"
Task: "T011 [P] [US1] Integration test for question unrelated to indexed documents `backend/tests/integration/test_rag_api.py`"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done, Developer(s) can work on User Story 1.

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
