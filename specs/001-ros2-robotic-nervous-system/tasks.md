# Tasks: Module 1 – ROS 2 Robotic Nervous System

**Input**: Design documents from `/specs/001-ros2-robotic-nervous-system/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are NOT requested in this specification. Tasks focus on content creation and validation.

**Organization**: Tasks are grouped by user story (chapter) to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Chapter 1, US2=Chapter 2, US3=Chapter 3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus content**: `Humain-robotic-book/docs/module-1-ros2/`
- **ROS 2 validation workspace**: `ros2_code_examples/src/module1_examples/`
- **Diagrams**: `Humain-robotic-book/docs/module-1-ros2/assets/`
- **Specs**: `specs/001-ros2-robotic-nervous-system/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and workspace structure for Module 1

- [ ] T001 Create module directory structure in Humain-robotic-book/docs/module-1-ros2/
- [ ] T002 Create assets subdirectory in Humain-robotic-book/docs/module-1-ros2/assets/
- [ ] T003 [P] Create _category_.json metadata file in Humain-robotic-book/docs/module-1-ros2/_category_.json
- [ ] T004 [P] Create ROS 2 code validation workspace at ros2_code_examples/ with src/ subdirectory
- [ ] T005 [P] Create ROS 2 package module1_examples with package.xml in ros2_code_examples/src/module1_examples/
- [ ] T006 [P] Create setup.py for module1_examples package in ros2_code_examples/src/module1_examples/setup.py
- [ ] T007 [P] Create Python module directory ros2_code_examples/src/module1_examples/module1_examples/ with __init__.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core Docusaurus configuration that MUST be complete before chapter content creation

**⚠️ CRITICAL**: No chapter work can begin until this phase is complete

- [ ] T008 Update docusaurus.config.ts title and metadata for Physical AI & Humanoid Robotics book in Humain-robotic-book/docusaurus.config.ts
- [ ] T009 [P] Verify sidebars.ts autogeneration configuration in Humain-robotic-book/sidebars.ts
- [ ] T010 [P] Test Docusaurus build succeeds: run npm run build from Humain-robotic-book/ directory
- [ ] T011 [P] Test Docusaurus dev server: run npm run start and verify site loads at http://localhost:3000

**Checkpoint**: Foundation ready - chapter implementation can now begin in parallel

---

## Phase 3: User Story 1 - ROS 2 Nodes and Topic Communication (Priority: P1) 🎯 MVP

**Goal**: Create Chapter 1 teaching students ROS 2 node communication using topics with publisher/subscriber pattern

**Independent Test**: Student can run 4 code examples (publisher, subscriber, custom message, QoS demo), complete 3 exercises, and verify topics using `ros2 topic echo` and `ros2 node list` commands within 20 minutes

### Code Development for Chapter 1

**Note**: ALL code must be developed and tested in ROS 2 workspace BEFORE embedding in MDX

- [ ] T012 [P] [US1] Create publisher_example.py with minimal publisher (1Hz, std_msgs/String) in ros2_code_examples/src/module1_examples/module1_examples/publisher_example.py
- [ ] T013 [P] [US1] Create subscriber_example.py with minimal subscriber in ros2_code_examples/src/module1_examples/module1_examples/subscriber_example.py
- [ ] T014 [P] [US1] Create publisher_custom_message.py demonstrating message content modification in ros2_code_examples/src/module1_examples/module1_examples/publisher_custom_message.py
- [ ] T015 [P] [US1] Create qos_demo.py demonstrating RELIABLE vs BEST_EFFORT QoS in ros2_code_examples/src/module1_examples/module1_examples/qos_demo.py
- [ ] T016 [US1] Update setup.py with entry_points for all 4 examples in ros2_code_examples/src/module1_examples/setup.py
- [ ] T017 [US1] Build ROS 2 package: run colcon build --packages-select module1_examples from ros2_code_examples/
- [ ] T018 [US1] Test publisher example: run ros2 run module1_examples publisher and verify output with ros2 topic echo
- [ ] T019 [US1] Test subscriber example: run both nodes together and verify communication
- [ ] T020 [US1] Test QoS demo: verify QoS behavior with ros2 topic info
- [ ] T021 [US1] Validate PEP 8 compliance: run black --check and flake8 on all Python files

### Diagram Creation for Chapter 1

- [ ] T022 [P] [US1] Create ch1-node-architecture.svg diagram showing Node → Executor → Callbacks in Humain-robotic-book/docs/module-1-ros2/assets/ch1-node-architecture.svg
- [ ] T023 [P] [US1] Create ch1-topic-pubsub-flow.svg diagram showing Publisher → Topic → Subscriber with QoS in Humain-robotic-book/docs/module-1-ros2/assets/ch1-topic-pubsub-flow.svg

### Chapter 1 MDX Content Creation

- [ ] T024 [US1] Create 01-nodes-and-topics.mdx with frontmatter (id, title, sidebar_position: 1) in Humain-robotic-book/docs/module-1-ros2/01-nodes-and-topics.mdx
- [ ] T025 [US1] Write Introduction section (2-3 sentences) in 01-nodes-and-topics.mdx
- [ ] T026 [US1] Write Prerequisites section with admonition block listing ROS 2 Humble, Python 3.10+, required packages in 01-nodes-and-topics.mdx
- [ ] T027 [US1] Write Learning Objectives section with 5 measurable outcomes in 01-nodes-and-topics.mdx
- [ ] T028 [US1] Write Concept Explanation sections for nodes, topics, QoS (H2/H3 structure) in 01-nodes-and-topics.mdx
- [ ] T029 [US1] Embed Example 1 (publisher) with explanation, validated code block, execution commands, expected output, key lines in 01-nodes-and-topics.mdx
- [ ] T030 [US1] Embed Example 2 (subscriber) with full explanation and code in 01-nodes-and-topics.mdx
- [ ] T031 [US1] Embed Example 3 (custom message) with explanation in 01-nodes-and-topics.mdx
- [ ] T032 [US1] Embed Example 4 (QoS demo) with RELIABLE vs BEST_EFFORT comparison in 01-nodes-and-topics.mdx
- [ ] T033 [US1] Write Exercise 1 (Basic: modify message content) with objective, steps, verification, collapsible solution in 01-nodes-and-topics.mdx
- [ ] T034 [US1] Write Exercise 2 (Intermediate: change topic name) with complete instructions in 01-nodes-and-topics.mdx
- [ ] T035 [US1] Write Exercise 3 (Advanced: QoS experimentation) with QoS mismatch scenario in 01-nodes-and-topics.mdx
- [ ] T036 [US1] Write Common Errors section with 5 examples (Context not initialized, No messages received, ModuleNotFoundError, Node name conflict, KeyboardInterrupt handling) in 01-nodes-and-topics.mdx
- [ ] T037 [US1] Write Summary section with 3-5 bullet points recapping key concepts in 01-nodes-and-topics.mdx
- [ ] T038 [US1] Write References section with minimum 5 IEEE citations (ROS2Docs2023, rclpy2023, ROS2Concepts2023, QoSGuide2023, DDS2015) in 01-nodes-and-topics.mdx

### Validation for Chapter 1

- [ ] T039 [US1] Verify word count is 2500-3000 words using wc -w on 01-nodes-and-topics.mdx
- [ ] T040 [US1] Count code examples (target: 4) and citations (target: ≥5) in 01-nodes-and-topics.mdx
- [ ] T041 [US1] Verify all code blocks have language tags (```python, ```bash) in 01-nodes-and-topics.mdx
- [ ] T042 [US1] Verify all diagrams have alt text and load correctly in 01-nodes-and-topics.mdx
- [ ] T043 [US1] Run npm run build from Humain-robotic-book/ and verify no errors for Chapter 1
- [ ] T044 [US1] Preview chapter in browser at http://localhost:3000 and check formatting, links, exercises

**Checkpoint**: Chapter 1 complete - students can learn ROS 2 nodes and topics independently

---

## Phase 4: User Story 2 - Services for Request-Response Communication (Priority: P2)

**Goal**: Create Chapter 2 teaching students ROS 2 services for synchronous request-response patterns

**Independent Test**: Student can run 4 code examples (service server, client, timeout handling, custom logic), complete 3 exercises, and call services using both Python client and `ros2 service call` command within 15 minutes

### Code Development for Chapter 2

- [ ] T045 [P] [US2] Create service_server.py with AddTwoInts server in ros2_code_examples/src/module1_examples/module1_examples/service_server.py
- [ ] T046 [P] [US2] Create service_client.py with synchronous client using call_async in ros2_code_examples/src/module1_examples/module1_examples/service_client.py
- [ ] T047 [P] [US2] Create service_client_timeout.py demonstrating wait_for_service with timeout in ros2_code_examples/src/module1_examples/module1_examples/service_client_timeout.py
- [ ] T048 [P] [US2] Create service_custom_logic.py with unit conversion (or similar) logic in ros2_code_examples/src/module1_examples/module1_examples/service_custom_logic.py
- [ ] T049 [US2] Update setup.py entry_points for 4 new service examples in ros2_code_examples/src/module1_examples/setup.py
- [ ] T050 [US2] Build package: run colcon build --packages-select module1_examples from ros2_code_examples/
- [ ] T051 [US2] Test service server: run ros2 run module1_examples service_server and verify with ros2 service list
- [ ] T052 [US2] Test service client: run client with server running and verify correct response
- [ ] T053 [US2] Test timeout handling: run client without server and verify timeout message
- [ ] T054 [US2] Validate PEP 8 compliance for all service examples

### Diagram Creation for Chapter 2

- [ ] T055 [P] [US2] Create ch2-service-sequence.svg or Mermaid diagram showing Client → Request → Server → Response flow in Humain-robotic-book/docs/module-1-ros2/assets/ch2-service-sequence.svg (or embedded Mermaid)

### Chapter 2 MDX Content Creation

- [ ] T056 [US2] Create 02-services.mdx with frontmatter (id: services, sidebar_position: 2) in Humain-robotic-book/docs/module-1-ros2/02-services.mdx
- [ ] T057 [US2] Write Introduction explaining services vs topics in 02-services.mdx
- [ ] T058 [US2] Write Prerequisites section (completion of Chapter 1, example_interfaces package) in 02-services.mdx
- [ ] T059 [US2] Write Learning Objectives with 4 measurable outcomes in 02-services.mdx
- [ ] T060 [US2] Write Concept Explanation sections for services, synchronous vs asynchronous, .srv file structure in 02-services.mdx
- [ ] T061 [US2] Embed Example 1 (service server) with AddTwoInts code, explanation, execution, output in 02-services.mdx
- [ ] T062 [US2] Embed Example 2 (service client) with call_async pattern in 02-services.mdx
- [ ] T063 [US2] Embed Example 3 (timeout handling) with wait_for_service code in 02-services.mdx
- [ ] T064 [US2] Embed Example 4 (custom logic) demonstrating modified service behavior in 02-services.mdx
- [ ] T065 [US2] Write Exercise 1 (Basic: modify computation) with multiply instead of add in 02-services.mdx
- [ ] T066 [US2] Write Exercise 2 (Intermediate: timeout handling) with 10-second wait in 02-services.mdx
- [ ] T067 [US2] Write Exercise 3 (Advanced: custom service) with string reversal outline in 02-services.mdx
- [ ] T068 [US2] Write Common Errors section with 4 examples (service hangs, Future not callable, type mismatch, request fields not set) in 02-services.mdx
- [ ] T069 [US2] Write Summary section recapping services vs topics in 02-services.mdx
- [ ] T070 [US2] Write References section with minimum 5 IEEE citations in 02-services.mdx

### Validation for Chapter 2

- [ ] T071 [US2] Verify word count is 2000-2500 words in 02-services.mdx
- [ ] T072 [US2] Count code examples (target: 4) and citations (target: ≥5) in 02-services.mdx
- [ ] T073 [US2] Verify all code blocks have language tags in 02-services.mdx
- [ ] T074 [US2] Run npm run build and verify no errors for Chapter 2
- [ ] T075 [US2] Preview Chapter 2 in browser and test navigation from Chapter 1

**Checkpoint**: Chapter 2 complete - students can implement ROS 2 services independently

---

## Phase 5: User Story 3 - Humanoid URDF Basics (Priority: P3)

**Goal**: Create Chapter 3 teaching students URDF structure for humanoid robots with validation tools

**Independent Test**: Student can read a 4-link humanoid URDF, identify joint types and limits, modify joint values, validate with `check_urdf`, and visualize with `urdf_to_graphiz` within 10 minutes

### URDF File Creation for Chapter 3

**Note**: URDF examples are XML files, not Python code

- [ ] T076 [P] [US3] Create simple_humanoid.urdf with 4 links (base_link, torso, left_arm, right_arm, head) and 3 joints in ros2_code_examples/src/module1_examples/module1_examples/urdf/simple_humanoid.urdf
- [ ] T077 [P] [US3] Add XML comments to simple_humanoid.urdf explaining <robot>, <link>, <joint>, <visual>, <collision>, <inertial> sections
- [ ] T078 [P] [US3] Create simple_humanoid_modified.urdf with altered joint limits (e.g., shoulder -π to π) in ros2_code_examples/src/module1_examples/module1_examples/urdf/simple_humanoid_modified.urdf
- [ ] T079 [P] [US3] Create simple_humanoid_with_hands.urdf adding left_hand and right_hand links in ros2_code_examples/src/module1_examples/module1_examples/urdf/simple_humanoid_with_hands.urdf
- [ ] T080 [US3] Validate simple_humanoid.urdf: run check_urdf simple_humanoid.urdf and verify "Successfully Parsed XML"
- [ ] T081 [US3] Validate modified URDF files with check_urdf
- [ ] T082 [US3] Generate link tree diagram: run urdf_to_graphiz simple_humanoid.urdf and verify PDF output

### Diagram Creation for Chapter 3

- [ ] T083 [P] [US3] Create ch3-coordinate-frames.svg showing 2-3 link robot with X/Y/Z axes (red/green/blue) in Humain-robotic-book/docs/module-1-ros2/assets/ch3-coordinate-frames.svg
- [ ] T084 [P] [US3] Create ch3-urdf-link-tree.svg from urdf_to_graphiz output (convert PDF to SVG or redraw) in Humain-robotic-book/docs/module-1-ros2/assets/ch3-urdf-link-tree.svg

### Chapter 3 MDX Content Creation

- [ ] T085 [US3] Create 03-urdf-basics.mdx with frontmatter (id: urdf-basics, sidebar_position: 3) in Humain-robotic-book/docs/module-1-ros2/03-urdf-basics.mdx
- [ ] T086 [US3] Write Introduction explaining URDF purpose for humanoid robotics in 03-urdf-basics.mdx
- [ ] T087 [US3] Write Prerequisites section (ROS 2 Humble, liburdfdom-tools, graphviz) in 03-urdf-basics.mdx
- [ ] T088 [US3] Write Learning Objectives with 4 measurable outcomes in 03-urdf-basics.mdx
- [ ] T089 [US3] Write Concept Explanation sections for URDF structure, links, joints, coordinate frames, tree topology in 03-urdf-basics.mdx
- [ ] T090 [US3] Embed Example 1 (annotated simple humanoid URDF) with XML code block and comments in 03-urdf-basics.mdx
- [ ] T091 [US3] Embed Example 2 (joint limit modification) highlighting changed lines in 03-urdf-basics.mdx
- [ ] T092 [US3] Embed Example 3 (adding hand link) with new link/joint XML in 03-urdf-basics.mdx
- [ ] T093 [US3] Write Exercise 1 (Basic: identify joint types) with table format solution in 03-urdf-basics.mdx
- [ ] T094 [US3] Write Exercise 2 (Intermediate: modify joint limits) with check_urdf validation in 03-urdf-basics.mdx
- [ ] T095 [US3] Write Exercise 3 (Advanced: add hand link) with full XML snippet solution in 03-urdf-basics.mdx
- [ ] T096 [US3] Write Common Errors section with 5 examples (parse failed, circular dependency, invalid joint type, missing inertial, upper < lower limit) in 03-urdf-basics.mdx
- [ ] T097 [US3] Write Summary section recapping URDF concepts in 03-urdf-basics.mdx
- [ ] T098 [US3] Write References section with minimum 5 IEEE citations (URDFSpec, ROS2URDF, urdfdom, ROS2TF, research paper) in 03-urdf-basics.mdx

### Validation for Chapter 3

- [ ] T099 [US3] Verify word count is 2500-3000 words in 03-urdf-basics.mdx
- [ ] T100 [US3] Count XML examples (target: 3) and citations (target: ≥5) in 03-urdf-basics.mdx
- [ ] T101 [US3] Verify all code blocks have language tag (```xml) in 03-urdf-basics.mdx
- [ ] T102 [US3] Run npm run build and verify no errors for Chapter 3
- [ ] T103 [US3] Preview Chapter 3 in browser and test cross-chapter navigation

**Checkpoint**: All 3 chapters complete - Module 1 content ready for integration

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Module-wide improvements and final validation

- [ ] T104 [P] Create module landing page intro in Humain-robotic-book/docs/module-1-ros2/index.md with overview, prerequisites, estimated completion time
- [ ] T105 [P] Verify _category_.json has correct label "Module 1: ROS 2 Robotic Nervous System" and position: 2
- [ ] T106 [P] Test all internal links between chapters (Chapter 1 → 2, 2 → 3, Summary links)
- [ ] T107 [P] Run link checker on full build: npx linkinator build/ --recurse from Humain-robotic-book/
- [ ] T108 [P] Verify all 11 code examples run on fresh ROS 2 Humble install (re-test publisher, subscriber, services, URDF validation)
- [ ] T109 [P] Count total citations across all chapters (target: ≥15 total, ≥5 per chapter)
- [ ] T110 [P] Validate constitution compliance: check technical accuracy, reproducibility, citations, testability sections against plan.md Constitution Check
- [ ] T111 Verify success criteria from spec.md: SC-001 (20 min pub-sub), SC-002 (15 min service), SC-003 (10 min URDF read), SC-004 (5 min URDF modify), SC-006 (code runs), SC-007 (citations), SC-010 (4-6 hour total)
- [ ] T112 Run full Docusaurus build: npm run build from Humain-robotic-book/ and verify build time <2 minutes
- [ ] T113 Test mobile responsiveness: preview on mobile viewport and verify readable text, diagrams scale
- [ ] T114 [P] Update main site navigation if needed in docusaurus.config.ts (navbar items for Module 1)
- [ ] T115 Final review: read through all 3 chapters end-to-end for flow, consistency, clarity

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion (T001-T007) - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion (T008-T011)
  - Chapter 1 (Phase 3): Can start after Foundational
  - Chapter 2 (Phase 4): Can start after Foundational (independent of Chapter 1)
  - Chapter 3 (Phase 5): Can start after Foundational (independent of Chapters 1 & 2)
- **Polish (Phase 6)**: Depends on all 3 chapters being complete (T012-T103)

### User Story Dependencies

- **Chapter 1 (US1 - P1)**: No dependencies on other chapters - can start after Foundational (T011)
- **Chapter 2 (US2 - P2)**: No dependencies on Chapter 1 (mentions "completion of Chapter 1" in Prerequisites section of content, but implementation is independent) - can start after Foundational
- **Chapter 3 (US3 - P3)**: No dependencies on Chapters 1 or 2 (URDF is separate topic) - can start after Foundational

**All chapters are independently implementable and testable**

### Within Each Chapter

- **Code Development** → **Diagram Creation** (parallel) → **MDX Content Creation** → **Validation**
- Code must be tested before embedding in MDX (T017-T021 before T029-T032 for Chapter 1)
- All code examples embedded before writing exercises (exercises reference code)
- Validation tasks at end verify chapter completeness

### Parallel Opportunities

**Setup Phase (Phase 1)**:
- T003, T004, T005, T006, T007 can run in parallel (different files)

**Foundational Phase (Phase 2)**:
- T009, T010, T011 can run in parallel after T008 completes

**Chapter 1 (Phase 3)**:
- T012, T013, T014, T015 (code files) can run in parallel
- T022, T023 (diagrams) can run in parallel
- After code validated, content sections can be written in parallel by multiple authors

**Chapter 2 (Phase 4)**:
- T045, T046, T047, T048 (code files) can run in parallel
- T055 (diagram) independent

**Chapter 3 (Phase 5)**:
- T076, T077, T078, T079 (URDF files) can run in parallel
- T083, T084 (diagrams) can run in parallel

**Polish Phase (Phase 6)**:
- T104, T105, T106, T107, T108, T109, T110, T114 can run in parallel

**Cross-Chapter Parallelization**:
- Once Foundational complete, all 3 chapters (Phases 3, 4, 5) can be worked on simultaneously by different team members

---

## Parallel Example: Chapter 1 Code Development

```bash
# Launch all code file creation tasks together:
Task T012: "Create publisher_example.py..."
Task T013: "Create subscriber_example.py..."
Task T014: "Create publisher_custom_message.py..."
Task T015: "Create qos_demo.py..."

# Then sequentially:
Task T016: "Update setup.py entry_points" (depends on T012-T015 being written)
Task T017: "Build package" (depends on T016)
Task T018-T021: "Test each example" (depends on T017)
```

## Parallel Example: All Chapters Simultaneously

```bash
# After Foundational Phase (T011) completes:

Developer A works on Chapter 1 (T012-T044)
Developer B works on Chapter 2 (T045-T075)
Developer C works on Chapter 3 (T076-T103)

# Each chapter progresses independently
# Integration happens in Phase 6 (Polish)
```

---

## Implementation Strategy

### MVP First (Chapter 1 Only)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T011) - CRITICAL
3. Complete Phase 3: Chapter 1 (T012-T044)
4. **STOP and VALIDATE**: Test Chapter 1 independently
   - Run all 4 code examples in ROS 2
   - Preview chapter in Docusaurus
   - Verify students can complete exercises
5. Deploy/demo Chapter 1 as standalone educational content

### Incremental Delivery

1. Complete Setup + Foundational → Module structure ready
2. Add Chapter 1 → Test independently → Deploy/Demo (MVP - Students can learn nodes & topics!)
3. Add Chapter 2 → Test independently → Deploy/Demo (Students now know topics & services!)
4. Add Chapter 3 → Test independently → Deploy/Demo (Complete Module 1!)
5. Each chapter adds value without breaking previous chapters

### Parallel Team Strategy

With 3 developers:

1. Team completes Setup (Phase 1) together - 7 tasks
2. Team completes Foundational (Phase 2) together - 4 tasks
3. Once Foundational is done:
   - **Developer A**: Chapter 1 (33 tasks - T012 to T044)
   - **Developer B**: Chapter 2 (31 tasks - T045 to T075)
   - **Developer C**: Chapter 3 (28 tasks - T076 to T103)
4. All chapters integrate during Polish phase (T104-T115)

**Timeline Estimate** (with parallel work):
- Setup: 2-3 hours
- Foundational: 1-2 hours
- Chapters (parallel): 12-16 hours per chapter (36-48 hours with 3 devs)
- Polish: 3-4 hours
- **Total: ~20-25 hours of work (can complete in ~50-60 hours with 3 developers)**

---

## Task Summary

**Total Tasks**: 115
- **Phase 1 (Setup)**: 7 tasks
- **Phase 2 (Foundational)**: 4 tasks (BLOCKING)
- **Phase 3 (Chapter 1 - US1)**: 33 tasks
- **Phase 4 (Chapter 2 - US2)**: 31 tasks
- **Phase 5 (Chapter 3 - US3)**: 28 tasks
- **Phase 6 (Polish)**: 12 tasks

**Parallelizable Tasks**: 47 tasks marked [P] (~41% can run in parallel within phases)

**Independent Test Criteria**:
- **Chapter 1**: Student runs 4 examples, verifies with `ros2 topic echo/list`, completes exercises in 20 minutes
- **Chapter 2**: Student runs service server/client, tests with `ros2 service call`, completes exercises in 15 minutes
- **Chapter 3**: Student reads URDF, validates with `check_urdf`, modifies joints, visualizes with `urdf_to_graphiz` in 10 minutes

**Suggested MVP Scope**: Complete through Phase 3 (Chapter 1) - provides foundational ROS 2 knowledge (nodes, topics, pub-sub)

**Validation**: All tasks follow required checklist format: `- [ ] [TaskID] [P?] [Story?] Description with file path`

---

## Notes

- [P] tasks = different files, no dependencies within phase
- [Story] labels: [US1]=Chapter 1, [US2]=Chapter 2, [US3]=Chapter 3
- Each chapter is independently completable and testable
- Code examples MUST be tested in ROS 2 workspace before embedding in MDX
- Commit after each logical task group (e.g., all Chapter 1 code files, then MDX content)
- Stop at any checkpoint to validate chapter independently
- Docusaurus build must succeed after each chapter integration
- Avoid: embedding untested code, missing file paths, breaking auto-generated sidebar
