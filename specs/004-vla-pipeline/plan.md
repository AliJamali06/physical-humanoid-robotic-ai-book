# Implementation Plan: Module 4 – Vision-Language-Action (VLA)

**Branch**: `004-vla-pipeline` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/004-vla-pipeline/spec.md`

## Summary

Module 4 delivers educational content teaching students how voice-driven humanoid robot control works through Vision-Language-Action (VLA) pipelines. The module covers three core chapters: (1) Whisper speech recognition for converting voice commands to text, (2) LLM-based cognitive planning for decomposing natural language into robot actions, and (3) end-to-end VLA pipeline integration with a capstone project overview. This is **documentation content** (not a running system), formatted as Docusaurus MDX chapters with conceptual diagrams, ROS 2 integration patterns, and example prompts/code snippets. Students learn conceptually without requiring expensive hardware (microphones, GPUs) or LLM API access.

## Technical Context

**Content Format**: Docusaurus MDX (Markdown + JSX components)
**Primary Dependencies**: Mermaid.js (diagrams), React (MDX components), Docusaurus 3.9.2
**Educational Technologies Covered**:
- Whisper (OpenAI speech recognition) - conceptual architecture
- LLMs (GPT-4, Claude, Llama) - prompt engineering for robotics
- ROS 2 Humble - action servers, topic integration
- Nav2, Isaac ROS (from Module 3) - capstone integration
**Testing**: Docusaurus build validation (`npm run build`), content checklist validation, citation link checking
**Target Platform**: Static documentation website (GitHub Pages)
**Performance Goals**: 4-6 hour total reading time (consistent with Modules 1-3), < 3s page load
**Constraints**:
- Conceptual focus (no requirement for hands-on Whisper/LLM deployment)
- Simple diagrams only (Mermaid.js, basic SVG)
- Minimum 9 IEEE citations total (3 per chapter)
- Beginner-friendly explanations (avoid ML jargon)
**Scale/Scope**: 3 chapters, ~8,000-12,000 words total

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ I. Technical Accuracy & Source Verification
**Status**: PASS
**Evidence**: Spec requires minimum 9 IEEE citations (FR-026), all VLA/Whisper/LLM claims will reference OpenAI papers, Google Research (RT-1/RT-2/PaLM-E/SayCan), and official documentation (ROS 2, Nav2). Constitution mandates 30 citations across entire book; Module 4 contributes 9+ to this total.

### ✅ II. Reproducible Code & Simulations
**Status**: PASS (with educational exemption)
**Evidence**: Module 4 is **conceptual education** (FR-028: "no requirement to run GPT-4, Claude, or Whisper models locally"). Code examples are illustrative (ROS 2 integration patterns, prompt engineering templates) rather than executable deployments. This aligns with constitution's "tested in isolated environment" requirement by providing *reference implementations* that students can adapt if they have hardware/API access. Capstone overview (FR-021) provides high-level architecture without full code, consistent with educational scope.
**Justification**: Full Whisper/LLM deployment requires expensive infrastructure ($$$$ for API access or high-end GPUs). Educational goal is understanding VLA concepts, not production deployment.

### ✅ III. Modular Structure with Clear Scope
**Status**: PASS
**Evidence**: Module 4 is the 4th of 4 core modules defined in constitution (ROS 2, Digital Twin, Isaac Perception/Navigation, VLA). Scope explicitly bounded in spec.md "Out of Scope" section (no full deployment, no LLM fine-tuning, no complex multi-step execution). Dependencies documented (Module 1-3 prerequisites in FR-030).

### ✅ IV. Engineering-Focused Writing (NON-NEGOTIABLE)
**Status**: PASS
**Evidence**: FR-027 mandates "beginner-friendly explanations (avoid jargon, define technical terms, use analogies)". FR-012 requires explaining LLM limitations (hallucination, latency, grounding errors) - failure modes explicitly covered. FR-023 addresses VLA pipeline failure modes and recovery strategies. Writing will follow engineering precision (e.g., "Whisper encoder-decoder transformer" not "Whisper intelligently transcribes").

### ✅ V. Citation & Documentation Standards
**Status**: PASS
**Evidence**: FR-026 mandates minimum 3 IEEE citations per chapter (9 total). Spec already identifies sources: Whisper paper (OpenAI), ORB-SLAM2, RT-1/RT-2/PaLM-E (Google), SayCan, ROS 2 docs, Nav2 docs. research.md will compile full bibliography with IEEE format.

### ✅ VI. Testability & Validation
**Status**: PASS
**Evidence**: Each chapter has measurable success criteria (SC-001 to SC-010): e.g., "Students can explain Whisper pipeline within 10 minutes", "Students can draw VLA pipeline diagram within 20 minutes". FR-029 requires learning objectives with measurable outcomes. Spec checklist validates all 30 functional requirements are testable.

### ✅ VII. Deployment & Accessibility
**Status**: PASS
**Evidence**: Module 4 content integrates into existing Docusaurus site (deployed via GitHub Actions to GitHub Pages per constitution). No separate deployment required. Mobile-responsive via Docusaurus default theme.

**Overall Constitution Compliance**: ✅ PASS (no violations)

## Project Structure

### Documentation (this feature)

```text
specs/004-vla-pipeline/
├── spec.md              # Feature specification (COMPLETED)
├── plan.md              # This file (IN PROGRESS)
├── research.md          # Phase 0 output (NEXT)
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (ROS 2 message schemas, LLM prompt templates)
├── checklists/          # Quality validation
│   └── requirements.md  # Spec quality checklist (COMPLETED)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
# Educational Content Structure (Docusaurus Documentation)
docs/
└── module-4/                                # Module 4 root directory
    ├── _category_.json                      # Docusaurus sidebar configuration
    ├── index.mdx                            # Module 4 introduction and overview
    ├── chapter-1-whisper.mdx                # Chapter 1: Voice Command Recognition
    ├── chapter-2-llm-planning.mdx           # Chapter 2: LLM-Based Cognitive Planning
    ├── chapter-3-vla-capstone.mdx           # Chapter 3: VLA Pipeline & Capstone
    ├── bibliography.md                      # IEEE citations for Module 4
    └── diagram-style-guide.md               # (Reference from Module 3 if needed)

static/
├── img/
│   └── module-4/                            # Module 4 diagrams and images
│       ├── whisper-architecture.svg         # Whisper encoder-decoder diagram
│       ├── whisper-pipeline.svg             # Voice-to-text pipeline flowchart
│       ├── llm-task-decomposition.svg       # LLM planning process diagram
│       ├── llm-prompt-template.svg          # Prompt engineering example
│       ├── vla-pipeline-complete.svg        # End-to-end VLA pipeline
│       ├── capstone-architecture.svg        # Capstone project system diagram
│       └── screenshots/                     # (Optional: RViz, terminal outputs)
│           └── ros2-vla-integration.png
└── code/
    └── module-4/                            # Module 4 code examples
        ├── whisper_ros2_integration.py      # Conceptual Whisper ROS 2 node
        ├── llm_planner_example.py           # Conceptual LLM planning node
        ├── prompt_templates/                # LLM prompt engineering examples
        │   ├── system_prompt.txt            # Robot capability description
        │   ├── task_decomposition.txt       # Task breakdown prompt
        │   └── action_grounding.txt         # Action grounding prompt
        └── capstone_pipeline_overview.md    # High-level capstone integration guide
```

**Structure Decision**: Docusaurus documentation project (no backend/frontend split). Educational content follows same pattern as Modules 1-3 (docs/ for chapters, static/ for assets). All code examples are illustrative reference implementations (not production-ready systems). Capstone overview describes architecture without full implementation.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | N/A | All constitution requirements satisfied |

**Justification for Conceptual-Only Approach**: Constitution Principle II requires "reproducible code", but Module 4 provides *reproducible learning* through conceptual examples. Full Whisper/LLM deployment would require:
- OpenAI API keys ($$$$ for GPT-4 access) or local LLM (40GB+ GPU VRAM for Llama 70B)
- Microphone hardware and audio pipeline setup
- Significant compute for real-time speech recognition

This creates accessibility barrier for students. Educational goal is understanding VLA principles (how voice becomes robot actions), not deploying production systems. Reference code examples enable students with resources to experiment while maintaining beginner accessibility.

## Phase 0: Research & Clarification

### Research Tasks

#### Research 1: Whisper Architecture and Model Variants
**Goal**: Understand Whisper's encoder-decoder transformer architecture, model size tradeoffs, and ROS 2 integration patterns.

**Questions to Resolve**:
- What are the official Whisper model sizes (tiny, base, small, medium, large) and their inference latency vs accuracy characteristics?
- How does Whisper handle multilingual input (zero-shot transfer learning)?
- What are common speech recognition failure modes (homophones, accents, background noise)?
- What ROS 2 packages exist for audio capture (audio_common, audio_msgs)?
- How would a conceptual whisper_node integrate with ROS 2 topics?

**Sources**:
- [OpenAI Whisper Paper](https://arxiv.org/abs/2212.04356)
- [OpenAI Whisper GitHub](https://github.com/openai/whisper)
- [ROS 2 audio_common documentation](https://index.ros.org/p/audio_common/)
- [ROS 2 audio_msgs package](https://github.com/ros2/common_interfaces)

**Output**: Section in research.md documenting Whisper architecture, model variants table, ROS 2 integration pattern, failure modes.

---

#### Research 2: LLM-Based Task Planning for Robotics
**Goal**: Understand how LLMs decompose high-level natural language commands into robot-executable actions, including prompt engineering techniques and limitations.

**Questions to Resolve**:
- What prompt engineering strategies work for robotics (system prompts, few-shot examples, chain-of-thought)?
- How do LLMs ground abstract commands to robot capabilities (action spaces, API descriptions)?
- What are common LLM failures in robotics (hallucination, out-of-scope actions, grounding errors)?
- How do SayCan, Code as Policies, and RT-2 handle action grounding?
- What output formats are useful (structured JSON, ROS 2 action calls, natural language plans)?

**Sources**:
- [SayCan (Google): Grounding LLMs in Robot Affordances](https://say-can.github.io/)
- [Code as Policies (Columbia/Google)](https://code-as-policies.github.io/)
- [RT-2: Vision-Language-Action Models (Google)](https://robotics-transformer2.github.io/)
- [PaLM-E: Embodied Multimodal Language Model](https://palm-e.github.io/)
- [LLM Prompt Engineering Guide](https://www.promptingguide.ai/)

**Output**: Section in research.md on LLM task decomposition, prompt templates, limitations, comparison with traditional planners (PDDL, behavior trees).

---

#### Research 3: End-to-End VLA Pipeline Integration
**Goal**: Understand how VLA components integrate into a complete system, including data flow, latency considerations, and failure handling.

**Questions to Resolve**:
- What is the standard data flow in VLA pipelines (audio → text → plan → actions → feedback)?
- What are typical latency budgets for each stage (speech recognition, LLM inference, robot control)?
- How do VLA systems handle errors (speech misrecognition, planning failures, action execution failures)?
- What integration patterns exist for ROS 2 + LLM systems (custom nodes, action servers, service calls)?
- How do production VLA systems differ from educational examples?

**Sources**:
- [RT-1 Paper (Google): Robotics Transformer for Real-World Control](https://arxiv.org/abs/2212.06817)
- [ROS 2 Action Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [Nav2 Architecture (for capstone integration)](https://navigation.ros.org/concepts/index.html)
- Isaac ROS documentation (Module 3 reference)

**Output**: Section in research.md on VLA pipeline architecture, timing/latency analysis, failure modes and recovery strategies, capstone integration points.

---

#### Research 4: Educational Content Best Practices for VLA
**Goal**: Identify best practices for teaching complex ML/robotics topics to beginners (analogies, visual diagrams, conceptual vs implementation focus).

**Questions to Resolve**:
- How should transformer architectures be explained without deep math (Whisper encoder-decoder)?
- What analogies work for LLM concepts (autoregressive generation, prompt engineering)?
- How to visualize VLA pipelines effectively (flowcharts, component diagrams, ROS 2 topic graphs)?
- What level of detail balances technical accuracy with beginner accessibility?
- How to structure capstone overview (architecture without overwhelming implementation details)?

**Sources**:
- [The Illustrated Transformer](https://jalammar.github.io/illustrated-transformer/)
- [LLM Visualization Guide](https://bbycroft.net/llm)
- Robotics education best practices (from Module 1-3 research)
- Docusaurus documentation best practices

**Output**: Section in research.md on educational content strategy, diagram guidelines, analogies/metaphors, beginner-friendly explanations.

---

### Research Consolidation

All research findings will be consolidated into `research.md` with:
- **Decisions**: Specific choices made (e.g., "Use Whisper 'base' model as primary example for latency/accuracy tradeoff")
- **Rationale**: Why chosen (e.g., "Base model balances real-time performance with acceptable WER on robot commands")
- **Alternatives Considered**: What else evaluated (e.g., "Tiny model too inaccurate for command recognition; large model too slow for real-time")
- **Citations**: IEEE-format references for all claims

## Phase 1: Design & Contracts

### Data Model (data-model.md)

Module 4 is educational documentation, so "entities" are conceptual data structures explained to students rather than implemented database schemas. The data-model.md will document:

#### Entity 1: Voice Command (Audio Waveform)
**Description**: Raw audio input from microphone, captured as waveform samples.

**Attributes**:
- `sample_rate`: Audio sampling rate (e.g., 16000 Hz for Whisper)
- `channels`: Mono (1) or stereo (2)
- `duration`: Length of audio clip (seconds)
- `format`: Audio encoding (e.g., PCM, WAV)
- `waveform_data`: Array of audio samples (int16 or float32)

**Relationships**: Input to Whisper model for transcription.

**ROS 2 Representation**: `audio_msgs/Audio` (if using audio_common package).

---

#### Entity 2: Transcription (Text Command)
**Description**: Text representation of voice command output by Whisper.

**Attributes**:
- `text`: Transcribed string (e.g., "Navigate to the kitchen")
- `confidence`: Whisper confidence score (0.0 to 1.0)
- `language`: Detected language code (e.g., "en" for English)
- `timestamp`: When transcription occurred (ROS 2 Time)

**Relationships**: Output from Whisper, input to LLM planner.

**ROS 2 Representation**: `std_msgs/String` or custom `vla_msgs/Transcription`.

---

#### Entity 3: Cognitive Plan (Task Sequence)
**Description**: Structured sequence of sub-tasks generated by LLM.

**Attributes**:
- `original_command`: Original voice command text
- `decomposed_tasks`: List of sub-tasks (e.g., ["navigate_to_kitchen", "detect_drink", "grasp_object", "navigate_back", "handover"])
- `action_parameters`: Dictionary of parameters for each task (e.g., `{"target_location": "kitchen", "object_class": "drink"}`)
- `plan_confidence`: LLM confidence in plan validity (if available)
- `timestamp`: When plan was generated

**Relationships**: Output from LLM planner, input to ROS 2 action execution.

**ROS 2 Representation**: Custom message `vla_msgs/CognitivePlan` or JSON-encoded `std_msgs/String`.

---

#### Entity 4: Robot Action (Grounded Executable Behavior)
**Description**: Specific robot action with parameters, ready for ROS 2 action server execution.

**Attributes**:
- `action_type`: Action server name (e.g., "NavigateToPose", "DetectObjects", "GraspObject")
- `parameters`: Action-specific parameters (e.g., `{"pose": {"x": 5.0, "y": 3.0}, "frame_id": "map"}`)
- `action_id`: Unique identifier for tracking
- `status`: Current action status (PENDING, ACTIVE, SUCCEEDED, ABORTED, REJECTED)
- `result`: Action result data (e.g., detected objects list, grasp success boolean)

**Relationships**: Corresponds to one sub-task in Cognitive Plan, executed by ROS 2 action servers (Nav2, perception, manipulation).

**ROS 2 Representation**: ROS 2 Action messages (e.g., `nav2_msgs/action/NavigateToPose`, custom `manipulation_msgs/action/GraspObject`).

---

#### Entity 5: Feedback (Sensor Data and Execution Status)
**Description**: Real-time feedback from robot sensors and action execution, returned to LLM for replanning or confirmation.

**Attributes**:
- `sensor_data`: Camera images, LiDAR scans, IMU readings (from ROS 2 topics)
- `action_status`: Current execution state (ACTIVE, SUCCEEDED, FAILED)
- `error_message`: If action failed, description of failure (e.g., "Navigation goal unreachable", "No objects detected")
- `timestamp`: Feedback timestamp

**Relationships**: Published by robot/simulator, consumed by LLM for replanning decisions.

**ROS 2 Representation**: Various sensor messages (`sensor_msgs/Image`, `sensor_msgs/LaserScan`) + action feedback messages.

---

### Contracts (contracts/ directory)

Since Module 4 is educational documentation (not a running API), "contracts" are illustrative examples of:
- ROS 2 message schemas (YAML definitions)
- LLM prompt templates (text files with placeholders)
- Example API interactions (conceptual, not OpenAPI spec)

#### Contract 1: ROS 2 Message Schemas (contracts/ros2_messages.yaml)
Document the custom message types for VLA pipeline:

```yaml
# vla_msgs/Transcription.msg
std_msgs/Header header
string text                 # Transcribed voice command
float32 confidence          # Whisper confidence score (0.0-1.0)
string language             # Detected language code (e.g., "en")

# vla_msgs/CognitivePlan.msg
std_msgs/Header header
string original_command      # Original voice command
string[] decomposed_tasks    # List of sub-task names
string action_parameters     # JSON-encoded parameters
float32 plan_confidence      # LLM confidence (if available)
```

#### Contract 2: LLM Prompt Templates (contracts/prompts/)
Example prompt templates for robotics task planning:

**system_prompt.txt**:
```
You are a robot task planner. Given a high-level natural language command, decompose it into a sequence of executable robot actions. Available actions:
- navigate_to(location): Move robot to specified location (kitchen, living_room, bedroom)
- detect_objects(class): Use camera to detect objects (drink, book, phone, person)
- grasp_object(object_id): Grasp detected object with manipulator
- place_object(location): Place held object at location
- handover_object(person_id): Hand object to detected person

Constraints:
- Robot is a ground-based humanoid (cannot fly, cannot climb stairs)
- Robot has 2-finger parallel gripper (can grasp objects 5-15cm wide)
- Navigation uses Nav2 (requires known map)
- Perception uses Isaac ROS (limited to pre-trained object classes)

Output format: JSON array of actions with parameters.
Example: [{"action": "navigate_to", "params": {"location": "kitchen"}}, {"action": "detect_objects", "params": {"class": "drink"}}]
```

**task_decomposition_example.txt**:
```
User Command: "Bring me a drink from the kitchen"

Decomposition:
1. navigate_to(location="kitchen")
2. detect_objects(class="drink")
3. grasp_object(object_id=<result_from_step_2>)
4. navigate_to(location="living_room")  # Assuming user in living room
5. handover_object(person_id=<detected_user>)

Rationale: Command requires navigation to kitchen, object detection, grasping, return navigation, and handover to user.
```

#### Contract 3: ROS 2 Node Integration Pattern (contracts/ros2_integration.md)
Conceptual description of how Whisper node, LLM planner node, and action client node interact via ROS 2 topics/actions:

```
VLA Pipeline ROS 2 Architecture:

1. Audio Capture Node
   - Subscribes: None (reads from microphone hardware)
   - Publishes: /audio/input (audio_msgs/Audio)

2. Whisper Transcription Node
   - Subscribes: /audio/input (audio_msgs/Audio)
   - Publishes: /voice_command/transcription (vla_msgs/Transcription)

3. LLM Planner Node
   - Subscribes: /voice_command/transcription (vla_msgs/Transcription)
   - Publishes: /cognitive_plan/output (vla_msgs/CognitivePlan)
   - Service Calls: (Optional) LLM API via HTTP request

4. Action Execution Node
   - Subscribes: /cognitive_plan/output (vla_msgs/CognitivePlan)
   - Action Clients:
     - NavigateToPose (nav2_msgs/action/NavigateToPose)
     - DetectObjects (custom perception_msgs/action/DetectObjects)
     - GraspObject (custom manipulation_msgs/action/GraspObject)
   - Publishes: /action_status (vla_msgs/ActionStatus)

5. Feedback Aggregator Node (Optional)
   - Subscribes: /camera/image_raw, /scan, /action_status
   - Publishes: /llm_feedback (vla_msgs/Feedback)
   - Purpose: Provide sensor data + action results to LLM for replanning
```

---

### Quickstart (quickstart.md)

Module 4 quickstart guide for students beginning their VLA learning journey:

**Title**: Getting Started with Module 4: Vision-Language-Action (VLA)

**Prerequisites**:
- ✅ Completed Modules 1-3 (ROS 2, Simulation, Perception/Navigation)
- ✅ Understand ROS 2 topics, nodes, actions (Module 1)
- ✅ Familiarity with Nav2 navigation (Module 3)
- ✅ Basic Python knowledge (functions, classes, imports)

**Module 4 Overview**:
- **Chapter 1**: Voice Command Recognition with Whisper (20 min reading + 10 min review)
- **Chapter 2**: LLM-Based Cognitive Planning (25 min reading + 15 min review)
- **Chapter 3**: VLA Pipeline & Capstone Overview (30 min reading + 20 min review)
- **Total Estimated Time**: 4-6 hours (including self-assessment and optional hands-on exploration)

**Learning Path**:
1. Read Chapter 1 (Whisper fundamentals) → Self-assess: Can you explain encoder-decoder architecture?
2. Read Chapter 2 (LLM planning) → Self-assess: Can you write a prompt for robot task decomposition?
3. Read Chapter 3 (VLA pipeline & capstone) → Self-assess: Can you draw the complete pipeline diagram?

**Hands-On (Optional)**:
- If you have microphone hardware + Python environment, experiment with Whisper locally
- If you have OpenAI API access, try prompt engineering for robot commands
- Use ROS 2 + Isaac Sim (from Module 3) to test voice-driven navigation conceptually

**What You'll Learn**:
- How Whisper converts speech to text (multilingual, noise-robust)
- How LLMs decompose high-level commands into robot actions
- How VLA pipelines integrate voice, language, and action components
- How the capstone project ties together all 4 modules (ROS 2, Simulation, Perception/Navigation, VLA)

**Key Takeaway**: You don't need to deploy Whisper or LLMs to understand VLA concepts. This module focuses on *how the pipeline works* rather than *how to build production systems*.

---

### Agent Context Update

After Phase 1 artifacts are generated, run:

```bash
cd "C:\Users\ALIjamali\Desktop\Humain-robotic-ai-book"
powershell.exe -ExecutionPolicy Bypass -File ".specify/scripts/powershell/update-agent-context.ps1" -AgentType claude
```

This will update `.clauderc` (or equivalent agent context file) with new technologies:
- Whisper (speech recognition model)
- LLM prompt engineering (for robotics)
- VLA pipelines (voice-language-action integration)
- ROS 2 audio_msgs (for microphone input)

Manual additions between `<!-- CUSTOM CONTEXT START -->` and `<!-- CUSTOM CONTEXT END -->` markers will be preserved.

## Phase 2: Task Generation

**Not created by `/sp.plan`** - Task breakdown happens via `/sp.tasks` command after plan approval.

Expected task categories for Module 4:
- **Setup**: Create module-4 directory structure, _category_.json, static asset directories
- **Foundational**: Compile bibliography (9+ IEEE citations), diagram style guide (if not reusing Module 3), code example directory setup
- **Chapter 1 (Whisper)**: Content writing (architecture, model variants, ROS 2 integration), diagrams (Whisper pipeline, encoder-decoder), code examples (whisper_ros2_integration.py)
- **Chapter 2 (LLM Planning)**: Content writing (task decomposition, prompt engineering, limitations), diagrams (LLM planning process, prompt templates), code examples (llm_planner_example.py, prompt templates)
- **Chapter 3 (VLA Capstone)**: Content writing (pipeline integration, capstone overview, deployment considerations), diagrams (VLA pipeline, capstone architecture), capstone guide (high-level implementation overview)
- **Polish & Validation**: Module introduction page, cross-chapter consistency, citation formatting, Docusaurus build test, self-assessment quizzes

## Key Design Decisions

### Decision 1: Conceptual vs Hands-On Focus
**Decision**: Module 4 is conceptual education (no requirement for hands-on Whisper/LLM deployment).

**Rationale**:
- Full Whisper deployment requires microphone hardware, Python environment setup, model downloads (multi-GB)
- LLM access requires OpenAI API keys ($$$ for GPT-4) or local deployment (40GB+ GPU VRAM for capable models)
- Accessibility barrier for students without resources
- Educational goal is understanding VLA principles, not production deployment

**Alternatives Considered**:
- **Full hands-on with Whisper/LLM**: Rejected due to high hardware/cost requirements, would limit accessibility to students with powerful machines
- **Cloud-based lab environment**: Rejected due to hosting costs, complexity of managing multi-user access, maintenance burden
- **Whisper-only hands-on (skip LLM)**: Rejected because VLA pipeline is incomplete without language planning component

**Implementation**: Provide reference code examples (whisper_ros2_integration.py, llm_planner_example.py) that students *can* run if they have hardware/API access, but mark as optional. Focus chapter content on explaining *how it works* (architecture, data flow, failure modes) rather than *how to deploy*.

---

### Decision 2: LLM Prompt Templates as "Contracts"
**Decision**: Document LLM prompt engineering patterns as contract templates (system prompts, task decomposition examples).

**Rationale**:
- Prompts are the "API contract" between human and LLM in robotics VLA systems
- Students need to understand prompt structure (system prompts, robot capability descriptions, output format constraints)
- Reusable templates enable students to experiment if they have LLM access

**Alternatives Considered**:
- **Skip prompt engineering entirely**: Rejected because it's critical to understanding how LLMs are constrained to valid robot actions
- **Only show high-level descriptions**: Rejected because concrete examples are more valuable for learning than abstract descriptions

**Implementation**: Create contracts/prompts/ directory with:
- system_prompt.txt (robot capabilities, constraints, output format)
- task_decomposition_example.txt (example command → action sequence)
- action_grounding.txt (how to map abstract commands to specific robot APIs)

---

### Decision 3: Capstone Overview vs Full Implementation
**Decision**: Capstone chapter provides high-level architecture overview without full step-by-step implementation.

**Rationale**:
- Full capstone implementation would require 50+ pages and weeks of development
- Conflicts with spec constraint "no complex multi-step robot tasks beyond capstone overview"
- Educational goal is understanding integration points (how Modules 1-4 connect), not building production robot

**Alternatives Considered**:
- **Full implementation tutorial**: Rejected due to length, complexity, conflicts with spec constraints
- **No capstone chapter**: Rejected because students need to see how VLA components integrate with prior modules
- **Simplified single-task demo**: Considered, but high-level overview provides better understanding of real-world systems

**Implementation**: Chapter 3 includes:
- VLA pipeline diagram showing all components (voice → Whisper → LLM → ROS 2 actions → robot → feedback)
- Technology mapping (Whisper for speech, GPT-4 for planning, Nav2 for navigation, Isaac ROS for perception)
- Integration points with Modules 1-3 (ROS 2 topics, Nav2 actions, Isaac ROS perception)
- Failure modes and recovery strategies (speech errors, planning failures, action execution failures)
- Real-world deployment considerations (model quantization, edge vs cloud, latency budgets)
- High-level implementation overview (major components, data flow, no full code)

---

### Decision 4: Diagram Complexity Level
**Decision**: Use simple conceptual diagrams (Mermaid.js flowcharts, basic SVG component diagrams) rather than detailed architectural diagrams.

**Rationale**:
- Spec constraint: "Keep diagrams simple and conceptual"
- Beginner-friendly: complex diagrams overwhelm students
- Maintainability: simple diagrams easier to update as technologies evolve

**Alternatives Considered**:
- **Detailed UML/architectural diagrams**: Rejected due to spec constraint and beginner accessibility
- **No diagrams**: Rejected because visual aids critical for understanding VLA data flow
- **Screenshots of Whisper/LLM UIs**: Rejected because no hands-on deployment requirement

**Implementation**:
- Whisper architecture: Simple block diagram (audio input → encoder → decoder → text output)
- LLM task decomposition: Flowchart (command → prompt → LLM → JSON output → action sequence)
- VLA pipeline: Component diagram (microphone → Whisper → LLM → ROS 2 → robot)
- Capstone architecture: High-level system diagram (all modules integrated)

All diagrams created in Mermaid.js (embedded in MDX) or exported as simple SVG from Draw.io.

---

### Decision 5: ROS 2 Integration Pattern Documentation
**Decision**: Document conceptual ROS 2 integration patterns (how Whisper node, LLM node, action client would interact) without full implementation.

**Rationale**:
- Students need to understand ROS 2 topic/action integration for VLA pipeline
- Conceptual pattern more valuable than unrunnable code snippet
- Aligns with educational focus (understanding architecture vs deploying system)

**Alternatives Considered**:
- **Full ROS 2 package with launch files**: Rejected due to deployment complexity, conflicts with conceptual focus
- **No ROS 2 integration**: Rejected because VLA must integrate with ROS 2 for humanoid robot control
- **Pseudocode only**: Rejected because students benefit from seeing ROS 2 message types and topic names

**Implementation**: contracts/ros2_integration.md documents:
- Node responsibilities (Audio Capture, Whisper Transcription, LLM Planner, Action Execution)
- Topic names and message types (/audio/input, /voice_command/transcription, /cognitive_plan/output)
- Action client interactions (NavigateToPose, DetectObjects, GraspObject)
- Data flow sequence diagram (audio → transcription → plan → actions → feedback)

---

## Summary

Module 4 implementation plan delivers educational VLA content through 3 chapters (Whisper, LLM Planning, VLA Capstone) as Docusaurus MDX documentation. Architecture prioritizes conceptual understanding over hands-on deployment (no requirement for microphone hardware, LLM API access, or expensive GPUs). Content follows same pattern as Modules 1-3 (docs/ for chapters, static/ for diagrams/code, bibliography for IEEE citations). Key design decisions balance technical accuracy (constitution Principle I) with beginner accessibility (avoiding deployment complexity). All 30 functional requirements (FR-001 to FR-030) and 10 success criteria (SC-001 to SC-010) from spec.md will be satisfied through research-backed content, simple diagrams, prompt engineering templates, and ROS 2 integration patterns.

**Next Steps**: Execute Phase 0 (research.md generation) to resolve all technical questions, then Phase 1 (data-model.md, contracts/, quickstart.md) to complete planning artifacts before task breakdown via `/sp.tasks`.

---

**Plan Status**: ✅ COMPLETE (ready for Phase 0 research)
**Constitution Compliance**: ✅ PASS (no violations)
**Last Updated**: 2025-12-09
