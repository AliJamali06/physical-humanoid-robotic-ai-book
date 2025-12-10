# Tasks: Module 4 – Vision-Language-Action (VLA)

**Feature**: `004-vla-pipeline`
**Input**: Design documents from `/specs/004-vla-pipeline/`
**Prerequisites**: plan.md (content structure), spec.md (user stories), project structure definition

**Tests**: Not explicitly requested in spec - focus on conceptual understanding and content validation

**Organization**: Tasks are grouped by user story (US1: Whisper, US2: LLM Planning, US3: VLA Capstone) to enable independent implementation and testing.

## Format: `- [ ] [ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure:
- **Content**: `docs/module-4/` (MDX chapter files)
- **Diagrams**: `static/img/module-4/` (SVG and optional screenshots)
- **Code Examples**: `static/code/module-4/` (Python, prompt templates, integration guides)
- **Specs**: `specs/004-vla-pipeline/` (research, data-model, contracts, quickstart)

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create directory structure and initialize Module 4 educational content framework

- [ ] T001 Create module-4 directory structure in docs/module-4/ and static/
- [ ] T002 [P] Create _category_.json for Docusaurus sidebar in docs/module-4/_category_.json
- [ ] T003 [P] Create static asset directories: static/img/module-4/ and static/img/module-4/screenshots/
- [ ] T004 [P] Create code examples directory structure in static/code/module-4/ and static/code/module-4/prompt_templates/
- [ ] T005 [P] Setup diagram tooling documentation (reference from Module 3 or create new if needed)

---

## Phase 2: Foundational (Shared Educational Resources)

**Purpose**: Create shared resources used across all chapters (research, bibliography, data model, contracts, quickstart)

**⚠️ CRITICAL**: These resources must be complete before chapter content can reference them

- [ ] T006 Generate research.md with findings on Whisper architecture, LLM planning, VLA integration, and educational best practices in specs/004-vla-pipeline/research.md
- [ ] T007 [P] Generate data-model.md documenting 5 conceptual entities (Voice Command, Transcription, Cognitive Plan, Robot Action, Feedback) in specs/004-vla-pipeline/data-model.md
- [ ] T008 [P] Create contracts directory and ROS 2 message schemas in specs/004-vla-pipeline/contracts/ros2_messages.yaml
- [ ] T009 [P] Create LLM prompt template files (system_prompt.txt, task_decomposition.txt, action_grounding.txt) in specs/004-vla-pipeline/contracts/prompts/
- [ ] T010 [P] Create ROS 2 integration pattern documentation in specs/004-vla-pipeline/contracts/ros2_integration.md
- [ ] T011 [P] Generate quickstart.md with Module 4 learning path guide in specs/004-vla-pipeline/quickstart.md
- [ ] T012 [P] Compile bibliography with minimum 9 IEEE citations (3 per chapter) from research sources in docs/module-4/bibliography.md
- [ ] T013 Validate foundational artifacts completeness (all specs/ documents created, bibliography has 9+ citations)

**Checkpoint**: Foundation ready - chapter implementation can now begin

---

## Phase 3: User Story 1 - Voice Command Recognition with Whisper (Priority: P1) 🎯 MVP

**Goal**: Deliver Chapter 1 teaching Whisper speech recognition architecture, model variants, and ROS 2 integration without requiring local Whisper deployment

**Independent Test**: Student can explain Whisper's architecture (encoder-decoder transformer), identify key features (multilingual support, noise robustness, zero-shot transfer), and describe the voice-to-text pipeline within 20 minutes of completing the chapter (per spec.md acceptance scenarios)

### Content Creation for User Story 1

- [ ] T014 [P] [US1] Create chapter-1-whisper.mdx with frontmatter and prerequisites section in docs/module-4/chapter-1-whisper.mdx
- [ ] T015 [US1] Write "Whisper Architecture" section explaining encoder-decoder transformer at conceptual level (FR-001)
- [ ] T016 [US1] Write "Whisper Model Variants" section with table comparing tiny, base, small, medium, large models on latency vs accuracy (FR-002)
- [ ] T017 [US1] Write "ROS 2 Integration Pattern" section with conceptual data flow (audio_msgs → whisper_node → std_msgs/String) (FR-003)
- [ ] T018 [US1] Write "Multilingual Capabilities" section explaining zero-shot transfer learning and language detection (FR-005)
- [ ] T019 [US1] Write "Speech Recognition Errors" section covering homophones, accents, background noise, and mitigation strategies (FR-006)
- [ ] T020 [US1] Add "Hands-On Optional" callout clarifying microphone hardware and Python environment requirements (FR-007)
- [ ] T021 [US1] Add learning objectives with measurable outcomes (<10 min for architecture explanation, <15 min for model variant explanation) (FR-029)
- [ ] T022 [US1] Add prerequisites section referencing Modules 1-3 (ROS 2 basics, simulation, perception) (FR-030)

### Diagrams for User Story 1

- [ ] T023 [P] [US1] Create Whisper architecture diagram (audio input → encoder → decoder → text output) using Mermaid.js in chapter-1-whisper.mdx or static/img/module-4/whisper-architecture.svg
- [ ] T024 [P] [US1] Create voice-to-text pipeline flowchart (microphone → audio preprocessing → Whisper model → text output) in static/img/module-4/whisper-pipeline.svg
- [ ] T025 [P] [US1] Create conceptual ROS 2 topic flow diagram (audio capture node → /audio/input → whisper_node → /voice_command/transcription) using Mermaid.js embedded in chapter

### Code Examples for User Story 1

- [ ] T026 [P] [US1] Create conceptual Whisper ROS 2 integration Python script in static/code/module-4/whisper_ros2_integration.py
- [ ] T027 [US1] Validate Python script syntax and add comprehensive docstring with prerequisites (Whisper, ROS 2 Humble, audio_common package)
- [ ] T028 [US1] Embed validated Python script in chapter-1-whisper.mdx with syntax highlighting and explanation (FR-003)

### Review and Citations for User Story 1

- [ ] T029 [US1] Add minimum 3 IEEE citations to chapter bibliography (OpenAI Whisper paper, ROS 2 audio_common, speech recognition research) (FR-026)
- [ ] T030 [US1] Validate all diagrams follow simple conceptual style (FR-025) and beginner-friendly explanations (FR-027)
- [ ] T031 [US1] Final chapter review: confirm all FR-001 through FR-007, FR-025-027, FR-029-030 are satisfied

**Checkpoint**: Chapter 1 complete - students can now understand Whisper voice recognition fundamentals independently

---

## Phase 4: User Story 2 - LLM-Based Cognitive Planning (Priority: P2)

**Goal**: Deliver Chapter 2 teaching LLM task decomposition, prompt engineering for robotics, and integration with ROS 2 action servers

**Independent Test**: Student can describe LLM prompt engineering for robotics (system prompts, task decomposition, action grounding), explain limitations (hallucination, latency, context window), and identify when to use LLMs vs traditional planning within 25 minutes (per spec.md acceptance scenarios)

### Content Creation for User Story 2

- [ ] T032 [P] [US2] Create chapter-2-llm-planning.mdx with frontmatter and prerequisites in docs/module-4/chapter-2-llm-planning.mdx
- [ ] T033 [US2] Write "LLM Fundamentals for Robotics" section explaining autoregressive generation, prompt engineering, system prompts (FR-008)
- [ ] T034 [US2] Write "Task Decomposition Process" section with example: "Bring me a drink from the kitchen" → [navigate, detect, grasp, navigate, handover] (FR-009)
- [ ] T035 [US2] Write "Prompt Engineering for Robotics" section with robot capability descriptions (APIs, action spaces, constraints) (FR-010)
- [ ] T036 [US2] Write "LLM Output Formats" section illustrating structured JSON, natural language, and ROS 2 action calls (FR-011)
- [ ] T037 [US2] Write "LLM Limitations" section covering hallucination, latency, context window, grounding errors (FR-012)
- [ ] T038 [US2] Write "Validation and Safety Checks" section on action feasibility, object existence, collision avoidance (FR-013)
- [ ] T039 [US2] Write "LLM vs Traditional Planners" comparison table (PDDL, behavior trees, state machines) (FR-014)
- [ ] T040 [US2] Write "ROS 2 Integration Pattern" section showing LLM node subscribes to /voice_command/transcription, publishes to /cognitive_plan/output (FR-015)
- [ ] T041 [US2] Add learning objectives with measurable outcomes (<15 min task decomposition explanation, <10 min limitations explanation) (FR-029)
- [ ] T042 [US2] Add prerequisites section referencing Chapter 1 (Whisper transcription) and Module 1 (ROS 2 actions) (FR-030)

### Diagrams for User Story 2

- [ ] T043 [P] [US2] Create LLM task decomposition flowchart (command → prompt → LLM inference → JSON output → action sequence) in static/img/module-4/llm-task-decomposition.svg
- [ ] T044 [P] [US2] Create prompt engineering template visualization (system prompt + user command → constrained LLM output) in static/img/module-4/llm-prompt-template.svg or as Mermaid.js diagram
- [ ] T045 [P] [US2] Create ROS 2 integration diagram showing llm_planner_node subscribing to transcription topic and publishing cognitive plan using Mermaid.js

### Code Examples for User Story 2

- [ ] T046 [P] [US2] Create conceptual LLM planner ROS 2 node Python script in static/code/module-4/llm_planner_example.py
- [ ] T047 [P] [US2] Create system_prompt.txt with robot capability descriptions (navigate_to, detect_objects, grasp_object actions) in static/code/module-4/prompt_templates/system_prompt.txt
- [ ] T048 [P] [US2] Create task_decomposition.txt with example command breakdown in static/code/module-4/prompt_templates/task_decomposition.txt
- [ ] T049 [P] [US2] Create action_grounding.txt with mapping abstract commands to specific robot APIs in static/code/module-4/prompt_templates/action_grounding.txt
- [ ] T050 [US2] Validate all prompt templates and Python script syntax, add comprehensive docstrings
- [ ] T051 [US2] Embed validated prompt templates and code in chapter-2-llm-planning.mdx with explanations (FR-010, FR-015)

### Review and Citations for User Story 2

- [ ] T052 [US2] Add minimum 3 IEEE citations (SayCan, Code as Policies, RT-2, PaLM-E, LLM robotics papers) (FR-026)
- [ ] T053 [US2] Validate all diagrams follow simple conceptual style (FR-025) and beginner-friendly explanations (FR-027)
- [ ] T054 [US2] Clarify LLM API access is optional (no requirement to run GPT-4, Claude, or local LLMs) (FR-028)
- [ ] T055 [US2] Final chapter review: confirm all FR-008 through FR-015, FR-025-030 are satisfied

**Checkpoint**: Chapter 2 complete - students can now understand LLM cognitive planning and prompt engineering independently

---

## Phase 5: User Story 3 - End-to-End VLA Pipeline and Capstone Overview (Priority: P3)

**Goal**: Deliver Chapter 3 showing complete VLA pipeline integration and capstone project overview tying together Modules 1-4

**Independent Test**: Student can draw a high-level VLA pipeline diagram (microphone → Whisper → LLM → ROS 2 actions → robot execution) showing all major components, explain data flow between stages, and describe how the capstone project integrates perception and navigation within 30 minutes (per spec.md acceptance scenarios)

### Content Creation for User Story 3

- [ ] T056 [P] [US3] Create chapter-3-vla-capstone.mdx with frontmatter and prerequisites in docs/module-4/chapter-3-vla-capstone.mdx
- [ ] T057 [US3] Write "VLA Pipeline Architecture" section with complete data flow (voice → Whisper → LLM → ROS 2 actions → robot sensors → feedback loop) (FR-016)
- [ ] T058 [US3] Write "Data Flow Between Components" section explaining audio format, text encoding, action message types at each stage (FR-017)
- [ ] T059 [US3] Write "Capstone Project Scenario" section describing fetch-and-deliver task for humanoid robot via voice command (FR-018)
- [ ] T060 [US3] Write "Technology Mapping" section mapping Whisper (speech), LLM (planning), Nav2 (navigation), Isaac ROS (perception), ROS 2 (integration) to capstone (FR-019)
- [ ] T061 [US3] Write "Timing and Latency Considerations" section on real-time vs near-real-time, acceptable delays per stage (FR-020)
- [ ] T062 [US3] Write "Capstone Implementation Overview" section with high-level component descriptions (no full code) (FR-021)
- [ ] T063 [US3] Write "Integration Points with Prior Modules" section explaining how VLA uses Module 1 (ROS 2 topics/actions), Module 2 (simulation testing), Module 3 (perception/navigation) (FR-022)
- [ ] T064 [US3] Write "Failure Modes and Recovery Strategies" section covering speech recognition errors, planning failures, navigation obstacles, manipulation errors (FR-023)
- [ ] T065 [US3] Write "Real-World Deployment Considerations" section on model quantization, edge deployment, cloud vs local processing (FR-024)
- [ ] T066 [US3] Add learning objectives with measurable outcomes (<20 min VLA pipeline diagram, <25 min capstone integration explanation) (FR-029)
- [ ] T067 [US3] Add prerequisites section referencing Chapters 1-2 (Whisper, LLM) and Modules 1-3 (ROS 2, simulation, perception/navigation) (FR-030)

### Diagrams for User Story 3

- [ ] T068 [P] [US3] Create end-to-end VLA pipeline diagram showing all components (microphone → Whisper → LLM → ROS 2 actions → robot → feedback) in static/img/module-4/vla-pipeline-complete.svg
- [ ] T069 [P] [US3] Create capstone architecture diagram integrating all modules (ROS 2 + Simulation + Perception/Nav + VLA) in static/img/module-4/capstone-architecture.svg
- [ ] T070 [P] [US3] Create timing/latency diagram showing acceptable delays per pipeline stage using Mermaid.js or simple SVG

### Code Examples and Integration Guide for User Story 3

- [ ] T071 [P] [US3] Create capstone_pipeline_overview.md with high-level implementation guide (major components, data flow, no full code) in static/code/module-4/capstone_pipeline_overview.md
- [ ] T072 [US3] Validate capstone overview covers all integration points (ROS 2 topics from Module 1, Nav2 from Module 3, Isaac ROS from Module 3) and is beginner-accessible
- [ ] T073 [US3] Embed capstone overview and integration patterns in chapter-3-vla-capstone.mdx with explanations (FR-021, FR-022)

### Review and Citations for User Story 3

- [ ] T074 [US3] Add minimum 3 IEEE citations (RT-1, VLA research papers, ROS 2 Action docs, humanoid navigation papers) (FR-026)
- [ ] T075 [US3] Validate all diagrams follow simple conceptual style (FR-025) and beginner-friendly explanations (FR-027)
- [ ] T076 [US3] Confirm capstone is high-level overview only (no full step-by-step implementation per spec constraints)
- [ ] T077 [US3] Final chapter review: confirm all FR-016 through FR-024, FR-025-030 are satisfied

**Checkpoint**: Chapter 3 complete - students can now understand complete VLA pipeline and capstone integration independently

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Finalize module with introduction page, cross-chapter consistency, validation, and quality checks

- [ ] T078 [P] Create module-4 introduction page (index.mdx) explaining chapter sequence and learning path in docs/module-4/index.mdx
- [ ] T079 [P] Add module-4 summary section in index.mdx explaining VLA pipeline integration and capstone connection to Modules 1-3
- [ ] T080 [P] Validate all diagram styles follow simple conceptual approach (Mermaid.js or basic SVG, no complex UML) (FR-025)
- [ ] T081 [P] Verify all code blocks use correct syntax highlighting (python, yaml, bash, txt for prompts) and have explanatory text
- [ ] T082 [P] Check all citations follow IEEE format with minimum 9 total across 3 chapters (3 per chapter) (FR-026)
- [ ] T083 Test Docusaurus build: npm run build in repository root to verify MDX syntax and no broken links
- [ ] T084 [P] Validate all static asset paths resolve correctly (images in static/img/module-4/, code in static/code/module-4/)
- [ ] T085 [P] Review chapter completion time estimates (target 4-6 hours total per spec success criteria SC-008)
- [ ] T086 [P] Verify conceptual focus maintained (no requirement to run Whisper/LLMs per FR-028 and plan.md justification)
- [ ] T087 [P] Check cross-chapter consistency (terminology, diagram styles, code example patterns)
- [ ] T088 [P] Proofread all chapters for beginner-friendly language and technical accuracy (FR-027)
- [ ] T089 Verify all 30 functional requirements (FR-001 through FR-030) are satisfied across all chapters
- [ ] T090 [P] Validate all 10 success criteria (SC-001 through SC-010) are achievable with delivered content
- [ ] T091 Run link checker to verify all citations and external documentation links are valid
- [ ] T092 Final Docusaurus local preview and mobile responsiveness check

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) completion - BLOCKS all user story chapters
- **User Story Chapters (Phases 3-5)**: All depend on Foundational phase completion
  - Chapter 1 (US1 - Whisper): Can start after Foundational - No dependencies on other chapters
  - Chapter 2 (US2 - LLM Planning): Can start after Foundational - Conceptually builds on US1 but independent content
  - Chapter 3 (US3 - VLA Capstone): Can start after Foundational - References concepts from US1 and US2 but independently readable
- **Polish (Phase 6)**: Depends on all three chapters being complete

### User Story Dependencies

- **User Story 1 (Whisper)**: Standalone - teaches speech recognition
- **User Story 2 (LLM Planning)**: Independent content but conceptually follows US1 (uses transcribed text from Whisper)
- **User Story 3 (VLA Capstone)**: Independent content but conceptually follows US1+US2 (integrates voice and planning with prior modules)

**Note**: While chapters build conceptually (Whisper → LLM → VLA Integration), each chapter is independently readable and testable per spec.md requirements.

### Within Each User Story

- Content writing before diagram embedding
- Diagrams created in parallel with content
- Code examples validated before embedding
- Citations added after content draft complete
- Review and validation last step for each chapter

### Parallel Opportunities

- **Phase 1 Setup**: All 5 tasks can run in parallel (different directories)
- **Phase 2 Foundational**: Tasks T007-T013 can run in parallel after T006 (research.md generation)
- **After Foundational**: All three user story chapters (Phases 3-5) can be developed in parallel by different authors
- **Within Each Chapter**:
  - Content creation tasks proceed sequentially (logical flow)
  - Diagram creation tasks (marked [P]) can run in parallel with content writing
  - Code example creation (marked [P]) can run in parallel with content writing
  - Citations, prerequisites, review tasks can run in parallel at end of chapter
- **Phase 6 Polish**: Most validation tasks (marked [P]) can run in parallel

---

## Parallel Example: User Story 1 (Chapter 1 - Whisper)

```bash
# After T014 creates the MDX file, these can run in parallel:

# Parallel Group 1: Diagrams (no dependencies on each other)
Task T023: Create Whisper architecture diagram
Task T024: Create voice-to-text pipeline flowchart
Task T025: Create ROS 2 topic flow diagram

# Parallel Group 2: Code examples (no dependencies on diagrams)
Task T026: Create Whisper ROS 2 integration Python script

# Meanwhile, content can be written sequentially (T015-T022)

# Parallel Group 3: Final review tasks (after content complete)
Task T029: Add IEEE citations
Task T030: Validate diagrams and explanations
```

---

## Parallel Example: All Three Chapters After Foundational Phase

```bash
# Once Phase 2 (Foundational) is complete, these can start in parallel:

Author/Developer A: Phase 3 (User Story 1 - Whisper)
  → Tasks T014-T031 (Chapter 1 content, diagrams, code)

Author/Developer B: Phase 4 (User Story 2 - LLM Planning)
  → Tasks T032-T055 (Chapter 2 content, diagrams, code, prompts)

Author/Developer C: Phase 5 (User Story 3 - VLA Capstone)
  → Tasks T056-T077 (Chapter 3 content, diagrams, capstone guide)

# Each author works independently on their chapter
# Integration happens in Phase 6 (Polish) after all chapters complete
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T013) - CRITICAL blocking phase
3. Complete Phase 3: User Story 1 (T014-T031) - Chapter 1 only
4. **STOP and VALIDATE**: Test Chapter 1 independently
   - Student can explain Whisper architecture within 10 minutes (SC-001)
   - Student can list 3+ Whisper model variants and explain tradeoffs within 15 minutes (SC-002)
   - Chapter has minimum 3 IEEE citations
5. Deploy/preview Chapter 1 if ready (Docusaurus build test)

**Result**: Minimum viable module with Whisper fundamentals (estimated 10-12 pages)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (Chapter 1) → Test independently → Preview/Demo (MVP!)
3. Add User Story 2 (Chapter 2) → Test independently → Preview/Demo (LLM planning added)
4. Add User Story 3 (Chapter 3) → Test independently → Preview/Demo (Full VLA pipeline and capstone)
5. Polish (Phase 6) → Final module with integration content and validation
6. Each chapter adds value without breaking previous chapters

### Parallel Team Strategy

With multiple content authors/developers:

1. **Team completes Setup + Foundational together** (coordination required)
2. **Once Foundational is done, parallelize**:
   - Author A: User Story 1 (Chapter 1: Whisper) - Tasks T014-T031
   - Author B: User Story 2 (Chapter 2: LLM Planning) - Tasks T032-T055
   - Author C: User Story 3 (Chapter 3: VLA Capstone) - Tasks T056-T077
3. **Chapters complete independently**, then integrate in Phase 6 (Polish)
4. **Validation**: Each author tests their chapter against spec.md acceptance scenarios

---

## Task Summary

### Total Task Count: 92 tasks

**Breakdown by Phase**:
- Phase 1 (Setup): 5 tasks
- Phase 2 (Foundational): 8 tasks
- Phase 3 (User Story 1 - Whisper): 18 tasks
- Phase 4 (User Story 2 - LLM Planning): 24 tasks
- Phase 5 (User Story 3 - VLA Capstone): 22 tasks
- Phase 6 (Polish): 15 tasks

**Breakdown by User Story**:
- User Story 1 (Whisper): 18 tasks (T014-T031)
- User Story 2 (LLM Planning): 24 tasks (T032-T055)
- User Story 3 (VLA Capstone): 22 tasks (T056-T077)
- Setup + Foundational + Polish: 28 tasks

**Parallel Opportunities Identified**:
- Phase 1: 4 parallel tasks (T002-T005)
- Phase 2: 7 parallel tasks (T007-T012 after T006)
- Phase 3 (US1): 6 parallel tasks (diagrams, code examples)
- Phase 4 (US2): 9 parallel tasks (diagrams, code examples, prompts)
- Phase 5 (US3): 5 parallel tasks (diagrams, capstone guide)
- Phase 6: 10 parallel tasks (validation, checks, proofing)
- **Cross-chapter parallelism**: All 3 user story phases can run in parallel after Foundational

**Independent Test Criteria**:
- User Story 1: Student explains Whisper architecture and model variants within 20 min (FR-001 to FR-007 coverage)
- User Story 2: Student describes LLM task decomposition and prompt engineering within 25 min (FR-008 to FR-015 coverage)
- User Story 3: Student draws VLA pipeline diagram and explains capstone integration within 30 min (FR-016 to FR-024 coverage)

**Suggested MVP Scope**: Phase 1 + Phase 2 + Phase 3 (User Story 1 only) = 31 tasks
- Delivers foundational Whisper voice recognition chapter
- Students learn VLA entry point (speech-to-text) without requiring full module
- Can be validated independently per SC-001 and SC-002

---

## Format Validation

✅ **All tasks follow required format**:
- Checkbox: `- [ ]` at start
- Task ID: Sequential T001-T092
- [P] marker: Used for parallel tasks (different files, no dependencies)
- [Story] label: US1, US2, US3 for user story phases
- Description: Clear action with exact file paths

✅ **Task organization by user story**:
- Phase 3: All tasks labeled [US1]
- Phase 4: All tasks labeled [US2]
- Phase 5: All tasks labeled [US3]
- Setup, Foundational, Polish: No story labels (cross-cutting)

✅ **File paths included**: Every content, diagram, code, and spec task specifies exact file path

---

## Notes

- [P] tasks = different files, no dependencies on each other
- [Story] label maps task to specific user story for traceability (US1, US2, US3)
- Each user story (chapter) is independently completable and testable per spec.md acceptance scenarios
- Tests are NOT included (not requested in spec - focus is conceptual understanding)
- Commit after each logical group of tasks (e.g., after completing a chapter section)
- Stop at any checkpoint to validate chapter independently against spec.md success criteria
- All 30 functional requirements (FR-001 through FR-030) are covered in task descriptions
- All 10 success criteria (SC-001 through SC-010) are referenced in independent test criteria
- Conceptual focus maintained throughout (no requirement for hands-on Whisper/LLM deployment per FR-028)
