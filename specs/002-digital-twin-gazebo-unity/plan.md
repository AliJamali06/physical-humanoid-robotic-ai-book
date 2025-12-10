# Implementation Plan: Module 2 – The Digital Twin (Gazebo & Unity)

**Branch**: `002-digital-twin-gazebo-unity` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-digital-twin-gazebo-unity/spec.md`

## Summary

Create Module 2 of the Physical AI & Humanoid Robotics book covering simulation and digital twin fundamentals for robotics students. Module consists of 3 chapters delivered as MDX files in Docusaurus: (1) Gazebo Physics and World Building, (2) Unity High-Fidelity Rendering and ROS 2 Integration, (3) Sensor Simulation (LiDAR, Depth Camera, IMU). Each chapter includes runnable Gazebo/Unity examples, SDF configurations, C# scripts, diagrams, exercises, and IEEE-format citations. Technical approach: Write chapters as standalone MDX files, organize under `/docs/module-2-digital-twin/` directory, validate Gazebo SDF files and Unity scenes in test environments, ensure complementary physics (Gazebo) + visualization (Unity) workflow, and verify Docusaurus build.

## Technical Context

**Language/Version**: MDX for content; SDF/XML for Gazebo worlds; C# for Unity scripts; Bash for launch commands
**Primary Dependencies**: Docusaurus 4.x, Gazebo Fortress (or Garden/Harmonic), Unity 2022.3 LTS, ROS-TCP-Connector, URDF Importer package, RViz for visualization
**Storage**: Static MDX files in `Humain-robotic-book/docs/module-2-digital-twin/`; Gazebo .world/.sdf examples; Unity scene configurations
**Testing**: Docusaurus build validation, Gazebo world launch tests (`gz sim`), Unity scene build tests, ROS 2 topic verification, link checker, IEEE citation format validation
**Target Platform**: GitHub Pages (book hosting); readers with Gazebo Fortress + Unity 2022.3 LTS + ROS 2 Humble on Ubuntu 22.04 (GPU required for Unity rendering)
**Project Type**: Documentation/educational content (Docusaurus site) + simulation configurations
**Performance Goals**: Docusaurus build <2 min, Gazebo real-time factor ≥0.8, Unity scene setup <30 min per student (SC-002)
**Constraints**: Gazebo Fortress/Unity 2022.3 LTS compatibility; all examples runnable on Ubuntu 22.04; min 5 IEEE citations per chapter; beginner-friendly (Module 1 prerequisite assumed)
**Scale/Scope**: 3 chapters, ~25-35 pages total, 11 runnable examples (4 Gazebo SDF + 4 Unity C# + 3 sensor configs), 5-7 hour student completion time (SC-010)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Constitution Principles

**I. Technical Accuracy & Source Verification** ✅
- Plan requires minimum 5 IEEE citations per chapter (FR-034) citing official Gazebo/Unity/ROS-TCP-Connector docs
- Gazebo examples tested on Gazebo Fortress (FR-028); Unity examples tested on Unity 2022.3 LTS (FR-029)
- Sensor configurations validated with `ros2 topic echo` and RViz visualization (FR-024, FR-025)
- SDF files validated with Gazebo parser (world launches without errors)
- Quality checks include technical accuracy validation against gazebosim.org, docs.unity3d.com, ROS-TCP-Connector GitHub

**II. Reproducible Code & Simulations** ✅
- Gazebo world files include complete SDF structure: physics engine settings, model definitions, plugin configurations (FR-002, FR-003)
- Unity setup includes step-by-step installation (ROS-TCP-Connector via Package Manager, URDF Importer) (FR-011)
- Prerequisites section lists GPU requirements, software versions, environment setup (FR-030)
- All code includes inline comments explaining configuration options (FR-032)
- Common Errors section for troubleshooting (FR-033): wrong SDF syntax, Unity-ROS connection failures, missing ROS message types

**III. Modular Structure with Clear Scope** ✅
- Module 2 is second of 4 planned modules (aligns with constitution's 4-module structure)
- Explicitly scoped: Gazebo physics, Unity rendering, sensor simulation only - excludes full navigation stack, Isaac integration (spec input: "Not building")
- Each user story independently testable: P1 (Gazebo physics alone), P2 (Unity rendering alone), P3 (sensor simulation with Gazebo/RViz)
- Clear boundaries: No advanced Unity scripting, no game development, no full humanoid navigation
- Builds on Module 1 prerequisite (ROS 2 basics) per SC-010

**IV. Engineering-Focused Writing** ✅
- Concrete examples required: gravity vector [0, 0, -9.81], friction coefficients (mu, mu2), LiDAR ray count (FR-005, FR-020)
- Failure modes documented: physics instability with large time steps, Unity-ROS connection loss, LiDAR performance vs accuracy (Edge Cases section)
- Debugging guidance: Gazebo GUI wireframe view, contact visualization, ROS 2 introspection tools (FR-009, SC-009)
- No vague language: requirements use MUST, specify exact parameters (time limits, sensor ranges)

**V. Citation & Documentation Standards** ✅
- IEEE format mandated (FR-034): inline citations, bibliography section
- Version pinning: Gazebo Fortress, Unity 2022.3 LTS, ROS 2 Humble (FR-028, FR-029)
- Official docs prioritized: gazebosim.org, docs.unity3d.com, github.com/Unity-Technologies/ROS-TCP-Connector
- Minimum 5 citations per chapter (SC-007)
- Testing includes automated citation format validation

**VI. Testability & Validation** ✅
- Acceptance criteria in spec for each user story (20 min Gazebo world, 30 min Unity scene, 15 min sensor setup) (SC-001, SC-002, SC-003)
- Gazebo world validation: launches without errors, physics behaves correctly (SC-001, SC-004)
- Unity scene validation: ROS connection works, joint motion updates in real-time (SC-002)
- Sensor validation: `/scan`, `/imu`, depth camera topics publish correctly (SC-003, SC-009)
- Student exercises with verification methods (ros2 topic echo, RViz visualization)
- All examples tested on Gazebo Fortress and Unity 2022.3 LTS (SC-006)

**VII. Deployment & Accessibility** ✅
- Docusaurus hosted on GitHub Pages (constitution requirement)
- Mobile-responsive (Docusaurus default theme)
- Setup prerequisites clearly documented (GPU for Unity, software versions, Ubuntu 22.04)
- Build automated via npm scripts
- Module completion time 5-7 hours for students with Module 1 background (SC-010)

### Gates Summary
- ✅ All 7 constitution principles satisfied
- ✅ No violations requiring justification
- ✅ Module 2 aligns with 4-module architecture (2 of 4)
- ✅ Ready to proceed to Phase 0 research

## Overall Book Architecture

**Context**: User requested "Docusaurus book architecture sketch" and "section + chapter structure for all modules" to ensure consistency across the book.

### Four-Module Structure

The Physical AI & Humanoid Robotics book follows a progressive learning path with 4 core modules as defined in constitution:

```text
Physical AI & Humanoid Robotics Book
│
├── Introduction / Landing Page
│   ├── Book overview and learning path
│   ├── Prerequisites (Ubuntu 22.04, hardware requirements)
│   └── Target audience (robotics students, beginner-intermediate)
│
├── Module 1: ROS 2 Robotic Nervous System (COMPLETED SPEC/PLAN/TASKS)
│   ├── Chapter 1: Nodes and Topics
│   ├── Chapter 2: Services
│   └── Chapter 3: URDF Basics
│   └── Skills: Pub/sub communication, request-response, robot modeling
│
├── Module 2: Digital Twin (Gazebo & Unity) (CURRENT - PLANNING PHASE)
│   ├── Chapter 1: Gazebo Physics and World Building
│   ├── Chapter 2: Unity Rendering and ROS 2 Integration
│   └── Chapter 3: Sensor Simulation (LiDAR, Depth, IMU)
│   └── Skills: Physics simulation, visualization, sensor data generation
│
├── Module 3: Isaac Perception & Navigation (FUTURE)
│   ├── Chapter 1: NVIDIA Isaac Sim Setup and Camera Processing
│   ├── Chapter 2: Nav2 Integration and Global Planning
│   └── Chapter 3: Obstacle Avoidance and Local Planning
│   └── Skills: Computer vision, path planning, navigation stack
│
├── Module 4: Vision-Language-Action Robotics (FUTURE)
│   ├── Chapter 1: VLA Model Integration
│   ├── Chapter 2: Multimodal Planning (vision + language)
│   └── Chapter 3: Action Execution and Feedback
│   └── Skills: LLM-based robotics, embodied AI, task planning
│
└── Capstone Project: Integrated Humanoid Robot Pipeline
    ├── End-to-end workflow: Voice → Planning → Navigation → Manipulation
    ├── Combines all 4 modules (ROS 2 + Simulation + Perception + VLA)
    └── Deliverable: Working demo with video/logs
```

### Cross-Module Consistency Standards

To ensure uniform quality across all modules (1-4), the following standards apply:

#### Chapter Structure Template (ALL modules)
```markdown
# Chapter Title

**Frontmatter**: title, description, keywords, sidebar_position

## Prerequisites
- Software versions (exact: ROS 2 Humble, Gazebo Fortress, etc.)
- Prior modules required (e.g., Module 2 requires Module 1)
- Hardware requirements (GPU, RAM, specific sensors)

## Learning Objectives
- 3-5 measurable outcomes (e.g., "Create X in <Y minutes")
- Aligned with success criteria in spec.md

## Concept Explanation
- Engineering-focused: concrete examples, no vague language
- Architectural diagrams (SVG format)
- Technical justifications (WHY this approach)

## Code Examples
- Structure: Explanation → Full code → Execution command → Expected output
- Inline comments (WHY, not WHAT)
- Language tags: ```python, ```xml, ```bash, ```csharp
- Runnable on fresh install (no hidden dependencies)

## Exercises
- Format: Objective → Numbered steps → Verification method → Solution (collapsed)
- Difficulty levels: Basic, Intermediate, Advanced
- Time estimates based on pilot testing

## Common Errors
- 4-5 realistic error scenarios
- Symptoms → Root cause → Solution
- Debugging commands (ros2 topic list, gz topic -l, etc.)

## References
- Minimum 5 IEEE-format citations
- Official docs prioritized (ros.org, gazebosim.org, docs.unity3d.com, docs.nvidia.com/isaac)
- Bibliography section at chapter end
```

#### Code Quality Standards (ALL modules)
- **Python (ROS 2, Isaac)**: PEP 8 compliance, Black formatter, complete imports, docstrings
- **SDF/XML (Gazebo, URDF)**: Valid XML, check_urdf passes, commented parameters
- **C# (Unity)**: Unity naming conventions, inline comments for ROS integration logic
- **Bash (commands)**: All commands tested, include expected output, error handling shown

#### Diagram Conventions (ALL modules)
- **Format**: SVG (vector graphics for scalability, accessibility)
- **Style**: Consistent color scheme across all modules
  - ROS 2 nodes: Blue rectangles
  - Topics/messages: Green ovals
  - Services: Orange rectangles
  - Sensors: Purple circles
  - Physics/simulation: Gray backgrounds
- **Labels**: Readable font size (14pt minimum), arrows labeled with message types
- **Tools**: Inkscape, Figma, or Mermaid (for simple flowcharts)
- **Storage**: `docs/module-X-name/assets/` directory, co-located with chapters

#### Citation Pattern (ALL modules)
```markdown
Inline: ROS 2 uses DDS for middleware [ROS2Docs2023].
Gazebo simulates physics with ODE or Bullet engines [GazeboSim2023].

## References

[ROS2Docs2023] "ROS 2 Documentation: Humble," Open Robotics, docs.ros.org/en/humble/, 2023.
[GazeboSim2023] "Gazebo Sim Documentation," Gazebo, gazebosim.org/docs/fortress/, 2023.
[UnityROS2024] "ROS-TCP-Connector," Unity Technologies, github.com/Unity-Technologies/ROS-TCP-Connector, 2024.
```

#### Navigation and Sidebar (ALL modules)
- **Top-level**: Auto-generated sidebar from filesystem (`sidebars.ts: autogenerated`)
- **Module organization**: Each module in `docs/module-N-name/` directory
- **Chapter numbering**: Filename prefix for ordering (`01-chapter-name.mdx`, `02-...`)
- **Category metadata**: `_category_.json` in each module directory
  ```json
  {
    "label": "Module N: Name",
    "position": N+1,
    "collapsible": true,
    "collapsed": false,
    "link": {
      "type": "generated-index",
      "description": "Brief module overview"
    }
  }
  ```

#### Testing and Validation Workflow (ALL modules)

**Stage 1: Code Development and Validation (BEFORE embedding in MDX)**
- Develop all code examples in isolated test environments:
  - **Module 1**: `ros2_code_examples/` workspace (ROS 2 packages)
  - **Module 2**: `gazebo_worlds/` (Gazebo SDF), `unity_scenes/` (Unity projects)
  - **Module 3**: `isaac_examples/` (Isaac Sim scenes)
  - **Module 4**: `vla_integration/` (VLA model code)
- Execute and verify output matches expected behavior
- Run automated tests (pytest for Python, launch tests for ROS 2)
- For Gazebo: Verify `gz sim world.sdf` launches, physics behaves correctly
- For Unity: Build scene, test ROS-TCP-Connector connection, verify joint updates
- For sensors: Check ROS 2 topics publish (`ros2 topic echo /scan`, `/imu`)

**Stage 2: Content Writing (AFTER code validated)**
- Write MDX chapter following template structure
- Embed validated code (copy-paste from test environment, no modifications)
- Add explanatory text, diagrams, exercises
- Include Common Errors section based on testing experience

**Stage 3: Build and Link Validation**
- Run `npm run build` (must succeed, <2 min)
- Check for broken links (Docusaurus `onBrokenLinks: 'throw'`)
- Validate MDX syntax (Prettier check)
- Count citations (≥5 per chapter)

**Stage 4: Technical Review**
- Peer review by domain expert (ROS 2 for Module 1, Gazebo/Unity for Module 2, Isaac for Module 3, VLA for Module 4)
- Student pilot test (2-3 students, measure completion time)
- Verify against success criteria in spec.md

**Stage 5: Integration**
- Merge chapter to module branch
- Test cross-chapter navigation
- Verify module completion time (target: 4-7 hours per module)

#### Quality Gates (ALL modules)
- ✅ **Code Execution**: All examples run successfully in CI (GitHub Actions)
- ✅ **Build Success**: `npm run build` succeeds, no broken links
- ✅ **Citation Count**: Each chapter ≥5 IEEE-format citations
- ✅ **Constitution Compliance**: No principle violations without documented justification
- ✅ **Peer Review**: Technical accuracy validated by expert
- ✅ **Pilot Test**: Students complete within target time (±20%)

## Project Structure

### Documentation (this feature)

```text
specs/002-digital-twin-gazebo-unity/
├── plan.md              # This file
├── research.md          # Phase 0: Gazebo/Unity/ROS-TCP-Connector research
├── data-model.md        # Phase 1: Chapter structure, SDF/C# templates
├── contracts/           # Phase 1: Chapter content contracts
│   ├── chapter-1-gazebo-physics.md
│   ├── chapter-2-unity-rendering.md
│   └── chapter-3-sensor-simulation.md
├── quickstart.md        # Phase 1: Development workflow for content authors
├── spec.md              # Requirements (already created)
└── checklists/
    └── requirements.md  # Quality checklist (already created)
```

### Source Code (Docusaurus book content)

```text
Humain-robotic-book/
├── docs/
│   ├── intro.md                              # Landing page
│   ├── module-1-ros2/                        # Module 1 (existing, implemented separately)
│   │   ├── _category_.json
│   │   ├── 01-nodes-and-topics.mdx
│   │   ├── 02-services.mdx
│   │   ├── 03-urdf-basics.mdx
│   │   └── assets/
│   ├── module-2-digital-twin/                # NEW: Module 2 content
│   │   ├── _category_.json                   # Sidebar metadata
│   │   ├── 01-gazebo-physics-world-building.mdx    # Chapter 1
│   │   ├── 02-unity-rendering-ros2-integration.mdx # Chapter 2
│   │   ├── 03-sensor-simulation.mdx          # Chapter 3
│   │   └── assets/                           # Chapter diagrams/images
│   │       ├── gazebo-architecture-diagram.svg
│   │       ├── sdf-structure-example.svg
│   │       ├── physics-engine-components.svg
│   │       ├── unity-ros-tcp-connector-flow.svg
│   │       ├── urdf-unity-articulation-body.svg
│   │       ├── lidar-ray-casting-diagram.svg
│   │       ├── depth-camera-buffer-diagram.svg
│   │       └── imu-sensor-placement.svg
│   └── module-3-isaac/                       # Future: Module 3
│   └── module-4-vla/                         # Future: Module 4
├── src/
│   ├── components/                           # Future: interactive components
│   └── css/
│       └── custom.css                        # Styling customizations
├── static/
│   └── img/                                  # Logos, favicons
├── docusaurus.config.ts                      # Site config (update metadata)
├── sidebars.ts                               # Auto-generated navigation
├── package.json                              # Dependencies
└── tsconfig.json                             # TypeScript config

# Code validation workspace (separate from book content)
gazebo_worlds/                                # NEW: Gazebo SDF validation
├── module2_examples/
│   ├── basic_world_gravity.world             # From Chapter 1 FR-002
│   ├── collision_materials_demo.world        # From Chapter 1 FR-005
│   ├── sensor_lidar_robot.world              # From Chapter 3 FR-020
│   ├── sensor_depth_camera.world             # From Chapter 3 FR-022
│   ├── sensor_imu_robot.world                # From Chapter 3 FR-023
│   └── models/
│       ├── simple_box/
│       │   └── model.sdf
│       └── humanoid_robot/
│           └── model.sdf
└── README.md                                 # Gazebo setup instructions

unity_scenes/                                 # NEW: Unity project validation
├── Module2Examples/                          # Unity project
│   ├── Assets/
│   │   ├── Scenes/
│   │   │   ├── BasicRobotVisualization.unity # From Chapter 2 FR-012
│   │   │   └── JointMotionDemo.unity         # From Chapter 2 FR-015
│   │   ├── Scripts/
│   │   │   ├── ROSConnection.cs              # ROS-TCP-Connector setup FR-014
│   │   │   └── JointStateSubscriber.cs       # Joint motion script FR-015
│   │   ├── URDF/
│   │   │   └── simple_robot.urdf             # Imported robot FR-013
│   │   └── Materials/                        # PBR materials FR-016
│   ├── Packages/
│   │   └── manifest.json                     # ROS-TCP-Connector, URDF Importer
│   └── ProjectSettings/
└── README.md                                 # Unity setup instructions
```

**Structure Decision**:
- **Docusaurus book**: Add new `docs/module-2-digital-twin/` directory alongside `module-1-ros2/`
- **Code validation**: Separate workspaces for Gazebo (`gazebo_worlds/`) and Unity (`unity_scenes/`) to test examples before embedding in MDX
- **Sidebar navigation**: Auto-generated from filesystem using `_category_.json` (consistent with Module 1)
- **Assets**: Co-locate diagrams with chapters in `module-2-digital-twin/assets/`
- **Rationale**: Separation of content and validation environments prevents build artifact pollution; aligns with Module 1 pattern for consistency; supports incremental chapter delivery

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected. All requirements align with constitution principles.

## Phase 0: Research

**Objective**: Validate technical assumptions and document Gazebo/Unity/ROS-TCP-Connector patterns for educational content

**Research Tasks**:

1. **Gazebo Fortress capabilities**: Confirm support for:
   - SDF 1.9+ format (required for Fortress)
   - Physics engines: ODE (default), Bullet, DART (FR-003)
   - Sensor plugins: gpu_lidar, depth_camera, imu (FR-020, FR-022, FR-023)
   - ROS 2 bridge: ros_gz_bridge for topic mapping (FR-021)
   - GUI features: wireframe view, contact visualization, center of mass display (FR-009)
   - Command-line tools: `gz sim`, `gz model`, `gz topic` (FR-007)

2. **Unity 2022.3 LTS and ROS-TCP-Connector**: Document exact workflow:
   - ROS-TCP-Connector installation via Package Manager (git URL)
   - URDF Importer package installation and usage
   - ROSConnection component configuration (IP, port, protocol)
   - Message serialization/deserialization for JointState messages
   - ArticulationBody mapping from URDF joints (FR-013)
   - Coordinate system conversion: ROS Z-up to Unity Y-up (FR-012)
   - Unity API: Input system, Transform API, GameObject hierarchy (FR-017)

3. **Sensor simulation patterns**: Research technical details:
   - LiDAR ray-casting: horizontal/vertical samples, range limits, FOV (FR-020)
   - Depth camera: depth buffer rendering, clip planes, point cloud generation (FR-022)
   - IMU sensor: orientation (quaternion), angular velocity, linear acceleration (FR-023)
   - Noise models: Gaussian noise on ranges, IMU bias drift, sensor update rates (FR-027)
   - RViz visualization: LaserScan display, PointCloud2 display, TF frames (FR-024)

4. **Gazebo-Unity integration workflow**: Study complementary roles:
   - Gazebo for physics (gravity, collision, dynamics) → Unity for visualization (rendering, lighting, materials)
   - ROS 2 bridge for data exchange: joint states, sensor data, camera images
   - Synchronization: Can Gazebo and Unity run simultaneously? (Research if needed for advanced workflow)
   - Alternative: Use Gazebo for physics + sensors, Unity for replay/visualization (separate timeline)

5. **Citation format patterns**: Research IEEE style for:
   - Software documentation: Gazebo (Open Robotics), Unity (Unity Technologies)
   - GitHub repositories: ROS-TCP-Connector, URDF Importer
   - Example: `[GazeboFortress2023] "Gazebo Fortress Documentation," Open Robotics, gazebosim.org/docs/fortress/, 2023.`

**Deliverable**: `research.md` with findings, SDF/C# code snippets, configuration examples, and decision justifications

## Phase 1: Design

**Objective**: Define chapter structure, content contracts, and example templates

### Data Model (Chapter Structure)

**Deliverable**: `data-model.md` defining:

- **Chapter template schema**: Same as Module 1 for consistency
  - Frontmatter (title, description, keywords, sidebar_position)
  - Section order: Prerequisites → Learning Objectives → Concept Explanation → Code Examples → Exercises → Common Errors → References
  - Metadata fields (word count, estimated time, difficulty)

- **Gazebo SDF example template**:
  - Structure: Concept explanation → Full SDF code block → Launch command → Expected behavior (visual + terminal output)
  - Boilerplate: `<sdf version="1.9">`, `<world>`, `<physics>`, `<model>`, `<plugin>`
  - Annotations: XML comments explaining key parameters (gravity vector, collision geometry, sensor config)
  - Validation checklist: Launches without errors, physics behaves correctly, sensors publish to ROS 2 topics

- **Unity C# script template**:
  - Structure: Explanation → Full C# code → Unity setup steps → Execution verification
  - Boilerplate: `using RosMessageTypes`, `using Unity.Robotics.ROSTCPConnector`, MonoBehaviour class
  - Annotations: Inline comments for ROS connection, message handling, coordinate conversion
  - Validation checklist: Script compiles, ROS connection successful, joint/sensor updates work

- **Exercise template**: Same as Module 1
  - Format: Objective → Instructions (numbered steps) → Verification method (ros2 topic echo, RViz visualization, Unity inspector) → Solution (collapsed)
  - Difficulty levels: Basic (modify SDF parameter), Intermediate (add sensor), Advanced (integrate Gazebo + Unity)

- **Diagram conventions**: Same as Module 1
  - Tool: SVG (vector graphics)
  - Style: Consistent color scheme (see Overall Book Architecture section)
  - Labels: Font size 14pt+, arrows labeled with data types

### Contracts (Chapter Content Specifications)

**Deliverable**: `contracts/` directory with 3 files:

**`chapter-1-gazebo-physics.md`** (maps to FR-001 through FR-009):
- **Word count target**: 3000-3500 words
- **Code examples**: 4 total (SDF/XML)
  1. Basic world file with ground plane, gravity, light sources (FR-002)
  2. Physics parameters demonstration (gravity vector, time step, solver type) (FR-003)
  3. Collision vs visual geometry (annotated SDF with `<collision>` and `<visual>`) (FR-004)
  4. Material properties (friction coefficients, restitution) with interactive demo (FR-005)
- **Diagrams**: 3 required
  1. Gazebo architecture (physics engine, rendering, sensor simulation components) (FR-001)
  2. SDF structure hierarchy (world → model → link → joint/sensor)
  3. Collision geometry vs visual geometry (wireframe + textured views)
- **Exercises**: 3 exercises
  1. Create world with ramp, box, sphere; adjust friction to observe sliding (FR-008)
  2. Modify physics engine parameters (time step, iterations) and observe stability
  3. Add plugin for world (e.g., wind plugin, custom physics parameter)
- **Prerequisites list**: Gazebo Fortress (or Garden/Harmonic), Ubuntu 22.04, ROS 2 Humble, basic terminal skills
- **Learning objectives**: 5 measurable outcomes (e.g., "Create Gazebo world with custom gravity in <20 min" - SC-001)
- **Common errors**: 5 examples (invalid SDF syntax, missing closing tags, physics instability, collision geometry errors, plugin loading failures) (FR-009, FR-033)
- **Citations**: Minimum 5 (gazebosim.org/docs/fortress/, SDF specification, physics engine docs) (FR-034, SC-007)

**`chapter-2-unity-rendering.md`** (maps to FR-010 through FR-018):
- **Word count target**: 2500-3000 words
- **Code examples**: 4 total (C#)
  1. ROS-TCP-Connector setup script (ROSConnection component configuration) (FR-014)
  2. JointState subscriber script (receive /joint_states, update Unity ArticulationBody) (FR-015)
  3. Camera control script (orbit camera with mouse input, zoom with scroll) (FR-017)
  4. Lighting and material setup (code comments explaining Unity PBR workflow) (FR-016)
- **Diagrams**: 2 required
  1. Gazebo (physics) vs Unity (visualization) complementary roles (FR-010)
  2. ROS-TCP-Connector data flow (ROS 2 topic → TCP endpoint → Unity ROSConnection → C# subscriber)
  3. URDF to Unity ArticulationBody mapping (links as GameObjects, joints as ArticulationBody components) (FR-013)
- **Exercises**: 3 exercises
  1. Import 2-link robot URDF and verify hierarchy in Unity scene (FR-018)
  2. Set up ROS-TCP-Connector and test connection with ROS 2 publisher
  3. Create lighting setup (directional + point lights) and configure PBR materials
- **Prerequisites list**: Unity 2022.3 LTS, ROS-TCP-Connector package (git URL), URDF Importer package, Module 1 completion (ROS 2 basics) (FR-011)
- **Learning objectives**: 4 measurable outcomes (e.g., "Set up Unity scene with ROS connection in <30 min" - SC-002)
- **Common errors**: 5 examples (ROS-TCP-Connector connection timeout, URDF import errors, coordinate system mismatch, missing ROS message types, ArticulationBody configuration issues) (FR-033)
- **Citations**: Minimum 5 (Unity documentation, ROS-TCP-Connector GitHub, URDF Importer docs, Unity PBR tutorials) (SC-007)

**`chapter-3-sensor-simulation.md`** (maps to FR-019 through FR-027):
- **Word count target**: 3000-3500 words
- **Code examples**: 3 total (SDF sensor configurations)
  1. LiDAR sensor SDF (gpu_lidar with ray count, range, FOV, ROS 2 topic mapping) (FR-020, FR-021)
  2. Depth camera SDF (depth_camera with resolution, clip planes, point cloud output) (FR-022)
  3. IMU sensor SDF (imu with noise models, orientation/angular velocity output) (FR-023)
- **Diagrams**: 3 required
  1. Sensor simulation concepts: ray-casting (LiDAR), depth buffers (depth camera), rigid body dynamics (IMU) (FR-019)
  2. LiDAR ray pattern (horizontal/vertical samples, FOV visualization)
  3. RViz visualization setup (LaserScan, PointCloud2, IMU orientation displays) (FR-024)
- **Exercises**: 3 exercises
  1. Add LiDAR to robot, spawn obstacles, verify ranges with `ros2 topic echo /scan` and RViz (FR-025)
  2. Configure depth camera and visualize point cloud in RViz
  3. Add IMU sensor, rotate robot in Gazebo, monitor orientation changes on `/imu` topic
- **Prerequisites list**: Gazebo Fortress with sensor plugins, ROS 2 Humble, RViz, ros_gz_bridge package (FR-021, FR-024)
- **Learning objectives**: 4 measurable outcomes (e.g., "Add LiDAR sensor and visualize in RViz in <15 min" - SC-003)
- **Common errors**: 5 examples (LiDAR not publishing, depth camera black image, IMU incorrect orientation, RViz visualization not appearing, performance issues with high ray count) (FR-033, SC-009)
- **Citations**: Minimum 5 (Gazebo sensor plugin docs, ROS 2 sensor message types, RViz documentation, sensor simulation research papers) (SC-007)

### Quickstart (Development Workflow)

**Deliverable**: `quickstart.md` documenting:

- **Setup**: Same as Module 1 for Docusaurus; additional setup for Gazebo and Unity
  - Clone repo, install dependencies (`npm install`)
  - Install Gazebo Fortress: `sudo apt install ros-humble-ros-gz`
  - Install Unity 2022.3 LTS from Unity Hub
  - Source ROS 2: `source /opt/ros/humble/setup.bash`

- **Writing workflow**:
  1. Create branch for chapter (e.g., `002-digital-twin-gazebo-unity/chapter-1`)
  2. Write MDX file in `docs/module-2-digital-twin/`
  3. Follow chapter contract (word count, code examples, diagrams, exercises)
  4. Validate examples BEFORE embedding:
     - **Gazebo**: Test SDF files in `gazebo_worlds/`, verify launch and behavior
     - **Unity**: Build Unity project in `unity_scenes/`, test ROS connection and scripts
  5. Embed validated code in MDX (copy-paste, no modifications)
  6. Create diagrams, save in `assets/`
  7. Run local preview: `npm run start`
  8. Validate build: `npm run build`
  9. Run link checker, citation counter
  10. Commit, create PR with checklist

- **Testing workflow**:
  1. **Gazebo validation**: Create .world/.sdf files in `gazebo_worlds/module2_examples/`
     - Test launch: `gz sim basic_world_gravity.world`
     - Verify physics: Spawn models, observe gravity/collision
     - Check sensors: `ros2 topic list`, `ros2 topic echo /scan`
  2. **Unity validation**: Create Unity project in `unity_scenes/Module2Examples/`
     - Install packages: ROS-TCP-Connector (git URL), URDF Importer
     - Test ROS connection: Run ROS 2 publisher, verify Unity receives messages
     - Verify joint updates: Publish JointState messages, check ArticulationBody motion
  3. Edge cases: Test error scenarios from "Common Errors" section
  4. Copy validated code to MDX (no modifications after testing)

- **Review checklist**:
  - [ ] Chapter meets word count target
  - [ ] All code examples tested and runnable (Gazebo launches, Unity builds)
  - [ ] Minimum citations met (5 per chapter)
  - [ ] Diagrams render correctly (SVG format)
  - [ ] Exercises have verification methods (ros2 topic echo, RViz, Unity inspector)
  - [ ] Prerequisites complete and accurate
  - [ ] Learning objectives measurable
  - [ ] Common Errors section includes 5 examples
  - [ ] Docusaurus build succeeds
  - [ ] Links valid (no 404s)
  - [ ] Constitution compliance (7 principles checked)

## Phase 2: Implementation Phases

**Note**: Detailed tasks generated by `/sp.tasks` command (not in this plan)

**High-level implementation flow**:

### Phase 2.1: Structure Setup
- Configure Docusaurus for Module 2 (update `docusaurus.config.ts` if needed, add `_category_.json`)
- Create `docs/module-2-digital-twin/` directory
- Set up Gazebo validation workspace (`gazebo_worlds/module2_examples/`)
- Set up Unity validation workspace (`unity_scenes/Module2Examples/`)
- Create diagram asset directory structure

### Phase 2.2: Chapter 1 - Gazebo Physics and World Building (Priority: P1)
- Write chapter content following `chapter-1-gazebo-physics.md` contract
- Develop and test 4 SDF examples in `gazebo_worlds/`
- Create 3 diagrams (Gazebo architecture, SDF structure, collision vs visual)
- Write 3 exercises with verification methods
- Add 5+ IEEE citations
- Write "Common Errors" section
- Validate Docusaurus build

### Phase 2.3: Chapter 2 - Unity Rendering and ROS 2 Integration (Priority: P2)
- Write chapter content following `chapter-2-unity-rendering.md` contract
- Develop and test Unity project with 4 C# scripts
- Create 3 diagrams (Gazebo vs Unity roles, ROS-TCP-Connector flow, URDF mapping)
- Write 3 exercises
- Add 5+ IEEE citations
- Write "Common Errors" section
- Validate Docusaurus build

### Phase 2.4: Chapter 3 - Sensor Simulation (Priority: P3)
- Write chapter content following `chapter-3-sensor-simulation.md` contract
- Create and validate 3 sensor SDF configurations
- Generate 3 diagrams (sensor concepts, LiDAR ray pattern, RViz setup)
- Write 3 exercises
- Add 5+ IEEE citations
- Write "Common Errors" section
- Validate Docusaurus build

### Phase 2.5: Integration and Review
- Test cross-chapter navigation (Chapter 1 → 2 → 3)
- Verify all internal links (between chapters, to Module 1 prerequisites)
- Run full Docusaurus build (`npm run build`)
- Validate against success criteria (SC-001 through SC-010)
- Create module landing page (overview, prerequisites: Module 1 + GPU requirements, expected time: 5-7 hours)
- Update main site navigation (ensure Module 2 appears after Module 1)
- Final constitution compliance check (re-validate 7 principles)

## Testing Strategy

### Build Validation (Continuous)
**Tools**: npm scripts, Docusaurus CLI

- `npm run build` — Full static build (must succeed, <2 min per constitution)
- `npm run start` — Local dev server (hot reload for content changes)

**Checks**:
- ✅ No broken links (Docusaurus `onBrokenLinks: 'throw'` config)
- ✅ All MDX files parse correctly
- ✅ Frontmatter valid (YAML schema)
- ✅ Images/diagrams load (404 check)

### Gazebo Example Validation (Per Chapter)
**Tools**: Gazebo Fortress CLI, ROS 2 tools

- **Execution test**: Launch each .world/.sdf file in `gazebo_worlds/`, verify no errors
  ```bash
  gz sim basic_world_gravity.world  # Should launch GUI with ground plane and gravity
  ```
- **Physics validation**: Spawn models, observe expected behavior (objects fall, collide, slide based on friction)
- **Sensor validation**: Check ROS 2 topics publish correctly
  ```bash
  ros2 topic list  # Verify /scan, /imu, /camera/depth topics appear
  ros2 topic echo /scan  # Verify LaserScan data
  ```
- **SDF syntax validation**: Gazebo parser validates XML structure (no warnings/errors on launch)

**Checks**:
- ✅ All worlds launch without errors on Gazebo Fortress (Ubuntu 22.04)
- ✅ Physics behaves correctly (gravity [0, 0, -9.81], collision detection, friction effects)
- ✅ Sensor plugins load and publish to ROS 2 topics
- ✅ SDF files follow 1.9+ specification
- ✅ XML well-formed (no syntax errors)

### Unity Example Validation (Per Chapter)
**Tools**: Unity 2022.3 LTS, ROS-TCP-Connector

- **Build test**: Open Unity project, verify no compilation errors
- **ROS connection test**:
  1. Start ROS 2 Humble environment
  2. Run Unity scene with ROSConnection component
  3. Verify TCP connection established (Unity console shows "Connected to ROS")
- **JointState test**: Publish `/joint_states` from ROS 2, verify Unity ArticulationBody updates
  ```bash
  ros2 topic pub /joint_states sensor_msgs/msg/JointState ...
  ```
- **URDF import test**: Import simple robot URDF, verify GameObject hierarchy matches link structure

**Checks**:
- ✅ Unity project builds without errors on Unity 2022.3 LTS
- ✅ ROS-TCP-Connector establishes connection to ROS 2 Humble
- ✅ C# scripts compile and execute correctly
- ✅ Joint motion updates in real-time from ROS 2 messages
- ✅ URDF imports successfully with correct ArticulationBody mapping
- ✅ Coordinate system conversion (ROS Z-up → Unity Y-up) handled correctly

### Formatting Consistency (Automated)
**Tools**: Same as Module 1 (Prettier, custom scripts)

- **Code block syntax**: Verify language tags (` ```xml ` for SDF, ` ```csharp ` for Unity scripts, ` ```bash ` for commands)
- **Indentation**: MDX uses 2 spaces (Prettier), XML uses 2 spaces, C# uses 4 spaces (Unity convention)
- **Headings**: Hierarchy check (no H1 → H3 jumps)
- **Link format**: Markdown links, not raw URLs

**Checks**:
- ✅ Prettier validation (`npx prettier --check docs/module-2-digital-twin/**/*.mdx`)
- ✅ Consistent heading levels (script: validate-headings.js)
- ✅ Code block language tags present (script: validate-code-blocks.js)

### Technical Accuracy Validation (Manual + Automated)
**Process**:
1. **Automated**: Citation format checker (IEEE pattern, minimum 5 per chapter)
2. **Automated**: API signature verification (compare SDF against Gazebo spec, C# against ROS-TCP-Connector API)
3. **Manual**: Technical peer review by Gazebo/Unity expert
4. **Manual**: Student pilot test (2-3 students follow chapters, report issues)

**Checks**:
- ✅ Minimum 5 IEEE citations per chapter (SC-007)
- ✅ Citations link to official docs (gazebosim.org, docs.unity3d.com, github.com/Unity-Technologies/ROS-TCP-Connector)
- ✅ SDF examples match Gazebo Fortress specification
- ✅ C# examples use correct ROS-TCP-Connector API (compatible with ROS 2 Humble)
- ✅ Terminal commands accurate (`gz sim`, `ros2 topic echo`, Unity build commands)
- ✅ No outdated information (version pinning: Gazebo Fortress, Unity 2022.3 LTS, ROS 2 Humble)

### Quality Gates (Pre-Merge)

**Gate 1: Code Execution** (blocking)
- All Gazebo worlds launch successfully in CI environment (GitHub Actions with Gazebo Fortress)
- Unity project builds without errors (Unity Cloud Build or local validation)
- Sensor topics publish correctly (automated ROS 2 topic check)

**Gate 2: Build Success** (blocking)
- `npm run build` succeeds
- No broken links reported
- Build artifacts generated in `build/` directory

**Gate 3: Citation Count** (blocking)
- Each chapter has ≥5 IEEE-format citations
- All citations have corresponding bibliography entries

**Gate 4: Constitution Compliance** (blocking)
- Spec acceptance criteria met (SC-001 through SC-010)
- No principle violations without documented justification

**Gate 5: Peer Review** (recommended, non-blocking)
- Technical accuracy reviewed by Gazebo and Unity experts
- Student readability test (pilot reader feedback)

## Architectural Decisions

### AD-001: Gazebo and Unity Complementary Roles
**Decision**: Use Gazebo for physics simulation and Unity for visualization as separate, complementary tools; NOT as simultaneous integrated system

**Options Considered**:
1. **Separate roles** (chosen): Gazebo for physics + sensors, Unity for visualization/rendering
   - Pros: Clear separation of concerns; each tool used for its strength; easier to learn incrementally
   - Cons: Cannot visualize Gazebo physics in Unity in real-time (requires complex integration)
2. **Integrated system**: Gazebo physics + Unity rendering simultaneously via ROS 2 bridge
   - Pros: Real-time Unity visualization of Gazebo simulation
   - Cons: Complex setup; synchronization issues; advanced topic beyond beginner scope
3. **Unity-only with ArticulationBody**: Use Unity's ArticulationBody for physics simulation
   - Pros: Single tool, simpler setup
   - Cons: Unity physics less robust than Gazebo for robotics; loses Gazebo sensor simulation capabilities

**Rationale**:
- **Educational focus**: Chapter 1 teaches Gazebo physics independently; Chapter 2 teaches Unity visualization independently; Chapter 3 builds on Gazebo sensors
- **Complementary strengths**: Gazebo excels at physics accuracy and sensor simulation (LiDAR, depth camera, IMU); Unity excels at photorealistic rendering and user interaction
- **Simplified learning**: Students learn one tool at a time (P1: Gazebo alone, P2: Unity alone, P3: sensors in Gazebo)
- **Spec alignment**: User stories are independently testable (spec requirement)
- **Future integration**: Advanced students can integrate Gazebo + Unity via ROS 2 bridge (capstone project scope, not Module 2)

**Trade-offs Accepted**: No real-time Unity visualization of Gazebo simulation in Module 2 (acceptable for beginner content). Students understand theoretical integration via ROS 2 bridge but don't implement it.

### AD-002: SDF Format Version and Gazebo Distribution
**Decision**: Target Gazebo Fortress with SDF 1.9+ format; note compatibility with Garden/Harmonic in documentation

**Options Considered**:
1. **Gazebo Fortress + SDF 1.9+** (chosen)
   - Pros: Aligns with ROS 2 Humble LTS (Ubuntu 22.04); stable release; wide adoption in 2023-2025 timeframe
   - Cons: Requires version pinning; future Gazebo releases may have breaking changes
2. **Gazebo Classic 11 + SDF 1.6**
   - Pros: Legacy system, widely documented
   - Cons: Gazebo Classic deprecated in favor of Gazebo Sim (new architecture); outdated for 2025 book
3. **Latest Gazebo (Garden/Harmonic) + SDF 1.10**
   - Pros: Cutting-edge features
   - Cons: Rapid changes; may not align with ROS 2 Humble LTS; fewer resources for troubleshooting

**Rationale**:
- Constitution mandates version pinning (no "latest")
- Gazebo Fortress released alongside ROS 2 Humble (2022), part of LTS ecosystem
- SDF 1.9 specification stable and well-documented
- FR-028 explicitly allows noting compatibility with Garden/Harmonic (forward compatibility)
- Educational stability: Students in 2025 courses likely using Ubuntu 22.04 + Humble + Fortress

**Trade-offs Accepted**: Content lifespan limited to Gazebo Fortress era (~2022-2026). Acceptable given LTS support and educational focus. Future updates can target Harmonic if ROS 2 ecosystem shifts.

### AD-003: Unity Package Installation Method
**Decision**: Install ROS-TCP-Connector and URDF Importer via Unity Package Manager with git URLs; NOT via Unity Asset Store or manual download

**Options Considered**:
1. **Package Manager with git URLs** (chosen)
   - Pros: Official method per ROS-TCP-Connector docs; supports version pinning (git tags); reproducible
   - Cons: Requires git installed; longer URL to copy-paste
2. **Unity Asset Store**
   - Pros: Easy discovery, one-click install
   - Cons: ROS-TCP-Connector NOT on Asset Store; version control unclear
3. **Manual download (zip file from GitHub)**
   - Pros: No git dependency
   - Cons: Manual extraction into Assets/ folder; harder to version control; not standard Unity workflow

**Rationale**:
- ROS-TCP-Connector official docs specify Package Manager with git URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
- Unity 2022.3 LTS supports git package URLs natively
- Version pinning possible via git tags (e.g., `...git?path=...#v0.7.0`)
- Reproducibility: Students get exact package version
- FR-011 requires installation guide; git URL method is standard for Unity robotics packages

**Trade-offs Accepted**: Requires git installed on student machine (reasonable prerequisite for robotics development). Slightly longer installation command (acceptable for clarity).

### AD-004: Sensor Visualization Tool Choice
**Decision**: Use RViz (ROS 2 visualization tool) for sensor data visualization (LaserScan, PointCloud2, IMU); NOT Unity or Gazebo GUI

**Options Considered**:
1. **RViz** (chosen)
   - Pros: Standard ROS 2 tool; built-in displays for LaserScan, PointCloud2, TF, IMU; students will use in later modules (Nav2)
   - Cons: Separate application (not embedded in Gazebo or Unity)
2. **Gazebo GUI visualization**
   - Pros: Integrated into Gazebo; no separate tool
   - Cons: Limited sensor visualization capabilities; harder to inspect point clouds, laser scans
3. **Unity visualization**
   - Pros: Photorealistic rendering
   - Cons: Requires custom C# scripts for sensor data; premature integration (violates AD-001 separation)

**Rationale**:
- FR-024 explicitly requires RViz for LaserScan and PointCloud2 visualization
- RViz is standard tool in ROS 2 ecosystem (students need familiarity for Module 3: Nav2 integration)
- Built-in displays match ROS 2 message types exactly (sensor_msgs/LaserScan, sensor_msgs/PointCloud2, sensor_msgs/Imu)
- Educational value: Students learn professional ROS 2 workflow (simulate in Gazebo, visualize in RViz, inspect with `ros2 topic echo`)
- SC-003, SC-009 explicitly mention RViz for verification

**Trade-offs Accepted**: Students must run 2 applications (Gazebo for simulation, RViz for visualization). Acceptable workflow for ROS 2 robotics development.

### AD-005: Code Example Complexity Level
**Decision**: Beginner-friendly examples with extensive inline comments; prioritize clarity over brevity

**Options Considered**:
1. **Beginner-friendly with extensive comments** (chosen)
   - Pros: Accessible to students with Module 1 background; self-documenting; aligns with constitution "Engineering-Focused Writing"
   - Cons: More verbose code; experienced developers may find comments redundant
2. **Concise professional-style code**
   - Pros: Shorter, cleaner code; mirrors real-world robotics projects
   - Cons: Harder for beginners; requires separate explanatory text
3. **Interactive tutorials with step-by-step builds**
   - Pros: Highly educational
   - Cons: Book format not interactive; would require video/GIF supplements

**Rationale**:
- Target audience: Beginner-intermediate robotics students (spec input)
- Constitution Principle IV: Engineering-Focused Writing with "code comments explain WHY, not WHAT"
- FR-032: "All code (SDF, C# scripts) MUST include inline comments explaining key lines and configuration options"
- Example from Module 1: rclpy publisher includes `# Create publisher with QoS RELIABLE for guaranteed delivery`
- Examples should be learning tools, not production code

**Trade-offs Accepted**: Code examples longer than minimal professional code. Acceptable for educational content prioritizing learning over brevity.

## Risk Analysis and Mitigation

### Risk 1: Gazebo World Files Fail to Launch in CI
**Probability**: Medium | **Impact**: High (blocks merge)

**Scenario**: SDF files work locally but fail in GitHub Actions CI due to missing Gazebo plugins, GPU requirements, or environment differences.

**Mitigation**:
- Use official Gazebo Docker images for CI (`gazebosim/gazebo:fortress`)
- Test in headless mode for CI: `gz sim -s world.sdf` (server mode, no GUI)
- Document exact Gazebo Fortress version (e.g., `gazebo fortress 7.4.0`)
- Validate SDF syntax before committing (use `gz sdf -k world.sdf` to check)
- Fallback: Manual validation attestation if CI GPU unavailable (document in PR)

### Risk 2: Unity ROS-TCP-Connector Version Incompatibility
**Probability**: Medium | **Impact**: Medium (code examples break)

**Scenario**: ROS-TCP-Connector package updates break API, causing C# scripts to fail compilation or runtime.

**Mitigation**:
- Pin exact package version via git tag (e.g., `...git?path=...#v0.7.0`)
- Test with specified version in Unity 2022.3 LTS before embedding examples
- Document version in FR-029 and Prerequisites section
- Monitor ROS-TCP-Connector GitHub releases; update documentation if breaking changes
- Provide migration guide if version update needed (update FR-029, add note to chapter)

### Risk 3: Students Lack GPU for Unity Rendering
**Probability**: Low | **Impact**: Medium (students cannot complete Chapter 2)

**Scenario**: Students attempt Unity 2022.3 LTS without GPU, experience slow rendering or crashes.

**Mitigation**:
- Clearly document GPU requirement in Prerequisites (FR-030): "GPU with 2GB+ VRAM (RTX 3060 or equivalent)"
- Provide alternative: Use Unity without ROS-TCP-Connector for visualization-only exercises (no real-time ROS connection)
- Cloud option: Document using Unity Cloud Build or remote desktop with GPU (e.g., AWS, Paperspace)
- Lightweigh alternative: Mention RViz as alternative for URDF visualization (`urdf_tutorial` package) for students without GPU

### Risk 4: Gazebo-Unity Integration Confusion
**Probability**: High | **Impact**: Low (student confusion, not blocking)

**Scenario**: Students expect real-time Gazebo-Unity integration after reading Chapters 1-2; frustrated when not demonstrated.

**Mitigation**:
- AD-001 clearly documents separation: Gazebo for physics, Unity for visualization (separate roles)
- Add explicit callout in Chapter 2:
  ```mdx
  :::note Gazebo and Unity Integration
  This module teaches Gazebo and Unity **separately** for clarity. Advanced integration
  (real-time Unity visualization of Gazebo simulation) requires complex ROS 2 bridge setup
  and is covered in the Capstone Project (Module 4). For now, focus on understanding each
  tool's strengths independently.
  :::
  ```
- Chapter 2 FR-010 explicitly states "Unity's role in digital twin (visualization, user interaction) vs Gazebo's role (physics simulation)"
- Exercises demonstrate Unity receiving ROS 2 joint states (Chapter 2), but from test publisher, not Gazebo

### Risk 5: Sensor Simulation Performance Issues
**Probability**: Medium | **Impact**: Low (learning experience, not blocking)

**Scenario**: Students set LiDAR ray count to extreme values (10,000+ rays), Gazebo simulation slows to <0.1 real-time factor.

**Mitigation**:
- Edge Case documented in spec: "What happens when LiDAR ray count is set extremely high? → Simulation slows down significantly"
- FR-026 requires performance considerations: "ray count vs FPS for LiDAR, resolution vs memory for depth cameras, update rate impact on CPU usage"
- Exercise in Chapter 3: Have students experiment with ray count (100 → 1000 → 5000) and measure FPS
- Common Errors section: "LiDAR running at <10 FPS → Reduce ray count or horizontal/vertical samples"
- Provide recommended values: "For learning: 100-500 rays; For realistic simulation: 1000-2000 rays"

### Risk 6: Citation Sources Become Outdated or Broken
**Probability**: Medium | **Impact**: Low (maintenance burden, not blocking)

**Scenario**: Official Gazebo/Unity documentation URLs change, citations break.

**Mitigation**:
- Use permalink where possible (e.g., Gazebo Fortress docs: `gazebosim.org/docs/fortress/` instead of `/docs/latest/`)
- Run automated link checker in CI (Docusaurus config `onBrokenLinks: 'throw'`)
- Archive critical documentation (Wayback Machine) if link-rot suspected
- Periodic review (quarterly): Validate all citations still accessible
- Provide fallback: If official docs move, update bibliography and add redirect note

## Success Metrics (Mapped to Spec)

**From spec.md Success Criteria**:
- **SC-001**: Student creates Gazebo world with custom gravity and collision objects in <20 min → Measure: Pilot test timing for Chapter 1 exercise
- **SC-002**: Student configures Unity scene with ROS-TCP-Connector and visualizes robot joint motion in <30 min → Measure: Pilot test timing for Chapter 2 exercise
- **SC-003**: Student adds LiDAR sensor to robot, launches world, visualizes in RViz in <15 min → Measure: Pilot test timing for Chapter 3 exercise
- **SC-004**: Student modifies physics parameters (friction, restitution) in SDF and observes changes in <10 min → Measure: Chapter 1 exercise timing
- **SC-005**: 85% of students complete all chapter exercises on first attempt → Measure: Self-assessment survey or pilot test success rate
- **SC-006**: All Gazebo worlds launch without errors on Gazebo Fortress (Ubuntu 22.04); Unity examples run on Unity 2022.3 LTS → Measure: CI test pass rate
- **SC-007**: Each chapter contains minimum 5 IEEE-format citations → Measure: Automated citation counter
- **SC-008**: Student explains difference between physics simulation (Gazebo) and rendering (Unity) after Chapters 1-2 → Measure: Comprehension quiz (75% pass rate)
- **SC-009**: Student diagnoses common sensor simulation issues using ROS 2 introspection tools in <5 min → Measure: Exercise timing for troubleshooting tasks
- **SC-010**: Module completion time averages 5-7 hours for beginner student with Module 1 prerequisite → Measure: Pilot test total time

**Additional Plan Metrics**:
- Docusaurus build time: <2 minutes (constitution)
- Page load time: <3s on 3G (constitution)
- Gazebo world validation: 100% launch success rate (all .sdf files)
- Unity project validation: 100% build success rate
- Sensor topic validation: 100% publish rate (all sensors in Chapter 3)
- Link checker pass rate: 100% (no 404s)
- XML/C# syntax validation: 100% (no compilation errors)

## Next Steps

1. **Review and Approve Plan**: Obtain user approval for architectural decisions (especially AD-001: Gazebo-Unity separation)
2. **Execute Phase 0**: Create `research.md` (Gazebo Fortress API, Unity ROS-TCP-Connector workflow, sensor simulation details)
3. **Execute Phase 1**: Create `data-model.md`, `contracts/chapter-1-gazebo-physics.md`, `contracts/chapter-2-unity-rendering.md`, `contracts/chapter-3-sensor-simulation.md`, `quickstart.md`
4. **Run `/sp.tasks`**: Generate dependency-ordered task list for Module 2 implementation
5. **Implement Chapters**: Follow iterative workflow (Draft → Validate → Review → Merge)
6. **Final Validation**: Full constitution compliance check, success criteria verification
7. **Deploy**: Merge to main, trigger GitHub Pages deployment, update book navigation
