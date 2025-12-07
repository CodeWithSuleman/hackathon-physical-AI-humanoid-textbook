# Tasks: Capstone Project: Full Autonomous Humanoid (ROS 2/Isaac)

**Input**: Design documents from `/specs/005-capstone-autonomous-humanoid/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/, quickstart.md

## Format: `[ID] [P?] [Story] Description`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
-   Include exact file paths in descriptions

## Phase 1: Setup (Docusaurus Project)

**Purpose**: Initialize the Docusaurus project and configure its basic structure for the Capstone Project.

- [x] T001 Create the initial directory structure for Capstone content under `docs/05-capstone-autonomous-humanoid/`.
- [x] T002 Create the initial directory structure for Whisper demos under `examples/capstone_whisper_demos/`.
- [x] T003 Create the initial directory structure for LLM planning demos under `examples/capstone_llm_planning_demos/`.
- [x] T004 Create the initial directory structure for the full VLA demo under `examples/capstone_full_vla_demo/`.

## Phase 2: Foundational (Capstone Project Prerequisites)

**Purpose**: Ensure basic environment setup and dependencies are in place for Capstone Project content creation.

- [x] T005 Verify NVIDIA GPU with CUDA support is available and configured.
- [x] T006 Verify Linux operating system (Ubuntu 20.04/22.04 LTS recommended) is installed.
- [x] T007 Verify Docker and NVIDIA Container Toolkit are installed and configured.
- [x] T008 Verify NVIDIA Isaac Sim installation and functionality via Omniverse Launcher.
- [x] T009 Verify NVIDIA Isaac ROS development environment setup (Docker containers/workspaces).
- [x] T010 Verify ROS 2 (Humble/Iron/Rolling) installation and environment setup.
- [x] T011 Verify Nav2 installation and functionality.
- [x] T012 Verify Python 3.10+ installation and availability.
- [x] T013 Verify OpenAI API Key is available and configured as environment variable `OPENAI_API_KEY`, or local LLM setup is complete.
- [x] T014 Verify a working microphone is connected and configured for audio input.
- [x] T015 Install Python libraries for OpenAI Whisper (e.g., `openai`, `pydub`, `sounddevice`).
- [x] T016 Install Python libraries for LLM interaction (e.g., `openai`, `langchain`).

## Phase 3: User Story 1 - Understanding the Vision-Language-Action (VLA) Pipeline (Priority: P1) 🎯 MVP

**Goal**: Students understand the complete Vision-Language-Action (VLA) pipeline, specifically how voice commands are processed into LLM plans and then executed as ROS 2 actions to achieve autonomous humanoid robot behavior.

**Independent Test**: Students can verbally or diagrammatically explain the VLA pipeline, detailing each stage (Voice → LLM Plan → ROS 2 Actions) and the role of each component.

### Implementation for User Story 1

- [x] T017 [US1] Write content for Chapter 1 in `docs/05-capstone-autonomous-humanoid/01-voice-to-action.md`, covering Whisper basics + command extraction.

## Phase 4: User Story 2 - Voice-to-Command with Whisper (Priority: P1)

**Goal**: Students should be able to run simple demonstrations of OpenAI Whisper to convert spoken voice commands into text-based robot commands.

**Independent Test**: Students can successfully execute a provided Whisper demonstration script, speak a command, and observe accurate text transcription of their voice input.

### Implementation for User Story 2

- [x] T018 [US2] Create a simple Whisper voice-to-command demo script (`examples/capstone_whisper_demos/voice_to_command_demo.py`).
- [x] T019 [US2] Add instructions to `docs/05-capstone-autonomous-humanoid/01-voice-to-action.md` on how to set up and run the Whisper demo.

## Phase 5: User Story 3 - Cognitive Planning with LLMs: Natural Language to ROS 2 Action Sequence (Priority: P1)

**Goal**: Students should be able to translate natural language tasks into a sequence of ROS 2 actions using a Language Model (LLM) for cognitive planning.

**Independent Test**: Students can provide a natural language task to a demonstration script, and observe the LLM's output as a valid, executable sequence of ROS 2 actions.

### Implementation for User Story 3

- [x] T020 [US3] Write content for Chapter 2 in `docs/05-capstone-autonomous-humanoid/02-cognitive-planning-llm.md`, covering NL → ROS 2 action sequence.
- [x] T021 [US3] Create a conceptual LLM cognitive planning demo script (`examples/capstone_llm_planning_demos/nl_to_ros_action_demo.py`).
- [x] T022 [US3] Add instructions to `docs/05-capstone-autonomous-humanoid/02-cognitive-planning-llm.md` on how to set up and run the LLM planning demo.

## Phase 6: User Story 4 - Mini Capstone: Autonomous Humanoid Demonstration (Priority: P1)

**Goal**: Students will be able to run a mini-capstone example that demonstrates the full VLA pipeline, enabling an autonomous humanoid robot to perform a task involving planning, navigation, detection, and manipulation based on a high-level command.

**Independent Test**: Students can issue a high-level natural language command (e.g., "Find the red block and place it on the table"), and observe the simulated humanoid robot successfully executing the entire task through voice processing, LLM planning, and ROS 2 actions.

### Implementation for User Story 4

- [x] T023 [US4] Write content for Chapter 3 in `docs/05-capstone-autonomous-humanoid/03-mini-capstone.md`, covering autonomous humanoid (plan, navigate, detect, manipulate).
- [x] T024 [US4] Create a mini-capstone full VLA demo (`examples/capstone_full_vla_demo/mini_capstone_vla_demo.py`).
- [x] T025 [US4] Add instructions to `docs/05-capstone-autonomous-humanoid/03-mini-capstone.md` on how to set up and run the mini-capstone demo.

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final review, integration, and validation of the module content and examples.

- [x] T026 Review all book content (`docs/05-capstone-autonomous-humanoid/`) for clarity, accuracy, word count (2000-3000 words), and APA citations.
- [x] T027 Verify all examples (`examples/capstone_whisper_demos/`, `examples/capstone_llm_planning_demos/`, `examples/capstone_full_vla_demo/`) are runnable and produce expected outputs.
- [x] T028 Verify the Docusaurus site builds correctly with the new module content (`npm run build`).

---

## Dependencies & Execution Order

-   **Setup (Phase 1)** must be completed first.
-   **Foundational (Phase 2)** depends on Setup.
-   **User Stories (Phases 3, 4, 5, 6)** depend on Foundational and can be worked on in parallel with each other.
-   **Polish (Phase 7)** depends on all other phases being complete.

## Implementation Strategy

The implementation will follow an MVP-first approach, prioritizing User Story 1 (VLA Pipeline), User Story 2 (Whisper), User Story 3 (LLM Planning), and User Story 4 (Mini Capstone). Each user story will be developed and tested incrementally.
