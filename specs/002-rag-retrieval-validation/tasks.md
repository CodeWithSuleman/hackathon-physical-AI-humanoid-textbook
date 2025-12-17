---

description: "Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts."
---

# Tasks: RAG Retrieval & Pipeline Validation

**Input**: Design documents from `/specs/002-rag-retrieval-validation/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md

**Tests**: This feature implicitly requests tests to confirm relevance, so test tasks will be included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create `backend/src/retrieval` directory `backend/src/retrieval`
- [X] T002 Add `cohere` and `qdrant-client` to `backend/requirements.txt` `backend/requirements.txt`
- [X] T003 Install dependencies using `uv` from `backend/requirements.txt`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Implement `__init__.py` for the retrieval module `backend/src/retrieval/__init__.py`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Core Retrieval Functionality (Priority: P1) 🎯 MVP

**Goal**: A developer needs to retrieve relevant text chunks from a vector database using a natural language query.

**Independent Test**: Can be fully tested by providing a natural language query to a Python function and asserting that the returned chunks are relevant and include metadata.

### Implementation for User Story 1

- [X] T005 [P] [US1] Create the core `retrieve_chunks` function signature `backend/src/retrieval/main.py`
- [X] T006 [P] [US1] Implement embedding generation for the query using Cohere `backend/src/retrieval/main.py`
- [X] T007 [P] [US1] Implement querying the Qdrant vector database `backend/src/retrieval/main.py`
- [X] T008 [P] [US1] Format the retrieved chunks with content, metadata, and score `backend/src/retrieval/main.py`
- [X] T009 [US1] Integrate embedding generation and Qdrant querying into `retrieve_chunks` `backend/src/retrieval/main.py`
- [X] T010 [P] [US1] Create unit tests for `retrieve_chunks` (e.g., empty query) `backend/src/retrieval/test_retrieval.py`
- [X] T011 [P] [US1] Create unit tests for `retrieve_chunks` (e.g., no relevant chunks) `backend/src/retrieval/test_retrieval.py`
- [X] T012 [P] [US1] Create unit tests for `retrieve_chunks` (e.g., empty vector database) `backend/src/retrieval/test_retrieval.py`
- [X] T013 [US1] Implement main logic of `retrieve_chunks` to pass unit tests `backend/src/retrieval/main.py`
- [X] T014 [US1] Develop acceptance tests using predefined queries `backend/src/retrieval/test_retrieval.py`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T015 Run quickstart.md validation to ensure the example works `specs/002-rag-retrieval-validation/quickstart.md`
- [X] T016 Code cleanup and refactoring for the retrieval module `backend/src/retrieval/`
- [ ] T017 Ensure all new technologies are documented in `GEMINI.md` and `CLAUDE.md` `GEMINI.md`, `CLAUDE.md`

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
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks T001, T002, T003 can be executed sequentially as they are environment setup tasks.
- Foundational task T004 must be completed before User Story 1.
- Within User Story 1, tasks T005, T006, T007, T008, T010, T011, T012 can be implemented in parallel.
- Tasks T009, T013, T014 will integrate these components and are sequential.

---

## Parallel Example: User Story 1

```bash
# Core components for User Story 1 can be developed in parallel:
Task: "T005 [P] [US1] Create the core `retrieve_chunks` function signature `backend/src/retrieval/main.py`"
Task: "T006 [P] [US1] Implement embedding generation for the query using Cohere `backend/src/retrieval/main.py`"
Task: "T007 [P] [US1] Implement querying the Qdrant vector database `backend/src/retrieval/main.py`"
Task: "T008 [P] [US1] Format the retrieved chunks with content, metadata, and score `backend/src/retrieval/main.py`"

# Unit tests can also be developed in parallel:
Task: "T010 [P] [US1] Create unit tests for `retrieve_chunks` (e.g., empty query) `backend/src/retrieval/test_retrieval.py`"
Task: "T011 [P] [US1] Create unit tests for `retrieve_chunks` (e.g., no relevant chunks) `backend/src/retrieval/test_retrieval.py`"
Task: "T012 [P] [US1] Create unit tests for `retrieve_chunks` (e.g., empty vector database) `backend/src/retrieval/test_retrieval.py`"
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
