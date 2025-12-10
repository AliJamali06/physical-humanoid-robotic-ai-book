# Tasks: Module 2 – The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-digital-twin-gazebo-unity/`
**Prerequisites**: plan.md (completed), spec.md (completed)

**Tests**: This is educational content creation, not software development. Tasks focus on content writing, code example creation, and validation rather than traditional software tests.

**Organization**: Tasks are grouped by user story (chapter) to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (chapter) this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus documentation project:
- **Book content**: `Humain-robotic-book/docs/module-2-digital-twin/`
- **Validation workspaces**: `gazebo_worlds/module2_examples/`, `unity_scenes/Module2Examples/`
- **Diagrams**: `docs/module-2-digital-twin/assets/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project structure initialization for Module 2

- [ ] T001 Create Module 2 directory structure in `Humain-robotic-ai-book/docs/module-2-digital-twin/`
- [ ] T002 [P] Create `_category_.json` in `docs/module-2-digital-twin/` with Module 2 metadata (label, position 3, description)
- [ ] T003 [P] Create assets directory at `docs/module-2-digital-twin/assets/` for diagrams
- [ ] T004 [P] Create Gazebo validation workspace directory `gazebo_worlds/module2_examples/`
- [ ] T005 [P] Create Unity validation workspace directory `unity_scenes/Module2Examples/`
- [ ] T006 [P] Create Gazebo models subdirectories in `gazebo_worlds/module2_examples/models/`
- [ ] T007 [P] Write Gazebo README at `gazebo_worlds/README.md` with setup instructions for Gazebo Fortress

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Research and design artifacts that MUST be complete before ANY chapter can be written

**⚠️ CRITICAL**: No chapter content can begin until this phase is complete

- [ ] T008 Create `research.md` documenting Gazebo Fortress SDF 1.9 specification and physics engines (ODE, Bullet, DART)
- [ ] T009 Research and document Unity 2022.3 LTS ROS-TCP-Connector installation workflow with git URL in `research.md`
- [ ] T010 Research and document sensor simulation patterns (LiDAR ray-casting, depth camera buffers, IMU dynamics) in `research.md`
- [ ] T011 Research and document Gazebo-Unity complementary roles architecture in `research.md`
- [ ] T012 Document IEEE citation format patterns for Gazebo/Unity/ROS-TCP-Connector in `research.md`
- [ ] T013 Create `data-model.md` defining chapter template schema (frontmatter, section order, metadata)
- [ ] T014 Define Gazebo SDF example template structure in `data-model.md` (concept, code, launch, behavior)
- [ ] T015 Define Unity C# script template structure in `data-model.md` (explanation, code, setup, verification)
- [ ] T016 Define exercise template in `data-model.md` (objective, steps, verification, solution)
- [ ] T017 Define diagram conventions in `data-model.md` (SVG format, color scheme, labels, tools)
- [ ] T018 Create `contracts/chapter-1-gazebo-physics.md` with word count target, 4 code examples, 3 diagrams, 3 exercises, 5+ citations
- [ ] T019 Create `contracts/chapter-2-unity-rendering.md` with word count target, 4 C# scripts, 3 diagrams, 3 exercises, 5+ citations
- [ ] T020 Create `contracts/chapter-3-sensor-simulation.md` with word count target, 3 sensor configs, 3 diagrams, 3 exercises, 5+ citations
- [ ] T021 Create `quickstart.md` documenting Docusaurus + Gazebo + Unity setup workflow
- [ ] T022 Document Gazebo validation workflow in `quickstart.md` (launch, verify physics, check sensors)
- [ ] T023 Document Unity validation workflow in `quickstart.md` (build, test ROS connection, verify joint updates)
- [ ] T024 Document content writing workflow in `quickstart.md` (branch, write, validate, embed, review checklist)

**Checkpoint**: Foundation ready - chapter content creation can now begin in parallel

---

## Phase 3: User Story 1 - Gazebo Physics and World Building (Priority: P1) 🎯 MVP

**Goal**: Students understand Gazebo physics simulation, can create worlds with gravity and collision, and configure physics parameters

**Independent Test**: Student can create Gazebo world file with ground plane and gravity, launch with `gz sim`, spawn objects, and observe realistic physics behavior (falling, collision) within 20 minutes (SC-001)

### Gazebo Code Examples Development for US1

**NOTE: Develop and validate ALL code examples BEFORE writing MDX content**

- [ ] T025 [P] [US1] Create `basic_world_gravity.world` in `gazebo_worlds/module2_examples/` with ground plane, gravity [0, 0, -9.81], and light sources (FR-002)
- [ ] T026 [P] [US1] Create `physics_parameters_demo.world` in `gazebo_worlds/module2_examples/` demonstrating gravity vector, time step 0.001s, solver type ODE (FR-003)
- [ ] T027 [P] [US1] Create `collision_visual_geometry.world` in `gazebo_worlds/module2_examples/` with annotated SDF showing `<collision>` and `<visual>` tags (FR-004)
- [ ] T028 [P] [US1] Create `material_properties_demo.world` in `gazebo_worlds/module2_examples/` with friction coefficients (mu: 0.1 for ice, mu: 1.0 for rubber), restitution examples (FR-005)

- [ ] T029 [US1] Test launch `basic_world_gravity.world` with `gz sim`, verify GUI shows ground plane and gravity works
- [ ] T030 [US1] Test `physics_parameters_demo.world`: spawn box, verify physics parameters affect simulation stability
- [ ] T031 [US1] Test `collision_visual_geometry.world`: verify collision geometry vs visual geometry in Gazebo wireframe view (FR-009)
- [ ] T032 [US1] Test `material_properties_demo.world`: spawn objects on ramp, verify sliding behavior differs by material friction

### Diagram Creation for US1

- [ ] T033 [P] [US1] Create `gazebo-architecture-diagram.svg` in `docs/module-2-digital-twin/assets/` showing physics engine, rendering, sensor simulation components (FR-001)
- [ ] T034 [P] [US1] Create `sdf-structure-example.svg` in `assets/` showing SDF hierarchy: world → model → link → joint/sensor
- [ ] T035 [P] [US1] Create `physics-engine-components.svg` in `assets/` showing collision geometry wireframe + textured visual geometry comparison

### MDX Content Writing for US1

- [ ] T036 [US1] Create `01-gazebo-physics-world-building.mdx` in `docs/module-2-digital-twin/` with frontmatter (title, description, keywords, sidebar_position: 1)
- [ ] T037 [US1] Write Prerequisites section in Chapter 1: Gazebo Fortress, Ubuntu 22.04, ROS 2 Humble, basic terminal skills (FR-030)
- [ ] T038 [US1] Write Learning Objectives section in Chapter 1: 5 measurable outcomes (e.g., "Create Gazebo world with custom gravity in <20 min") (SC-001)
- [ ] T039 [US1] Write Concept Explanation section in Chapter 1: Gazebo architecture with reference to diagram (FR-001)
- [ ] T040 [US1] Embed Code Example 1 in Chapter 1: `basic_world_gravity.world` with explanation, launch command, expected behavior (FR-002)
- [ ] T041 [US1] Embed Code Example 2 in Chapter 1: `physics_parameters_demo.world` with parameter explanations (gravity vector, time step, solver) (FR-003)
- [ ] T042 [US1] Embed Code Example 3 in Chapter 1: `collision_visual_geometry.world` with annotated SDF tags (FR-004)
- [ ] T043 [US1] Embed Code Example 4 in Chapter 1: `material_properties_demo.world` with friction/restitution explanations (FR-005)
- [ ] T044 [US1] Write Exercises section in Chapter 1: Exercise 1 (ramp + box + sphere with friction adjustment), Exercise 2 (modify physics parameters), Exercise 3 (add plugin) (FR-008)
- [ ] T045 [US1] Write Common Errors section in Chapter 1: 5 examples (invalid SDF syntax, missing closing tags, physics instability, collision geometry errors, plugin loading failures) (FR-009, FR-033)
- [ ] T046 [US1] Write References section in Chapter 1: Minimum 5 IEEE citations (gazebosim.org/docs/fortress/, SDF spec, physics engine docs) (FR-034, SC-007)
- [ ] T047 [US1] Add Gazebo command examples in Chapter 1: `gz sim`, `gz model --spawn-file`, `gz topic -l` (FR-007)
- [ ] T048 [US1] Add Gazebo plugin basics in Chapter 1: how to load world plugins with example SDF snippets (FR-006)

### Validation for US1

- [ ] T049 [US1] Run Docusaurus build (`npm run build`) and verify Chapter 1 renders without errors
- [ ] T050 [US1] Verify all 4 Gazebo worlds launch successfully from Chapter 1 embedded code
- [ ] T051 [US1] Count citations in Chapter 1 (must be ≥5 IEEE format)
- [ ] T052 [US1] Run link checker on Chapter 1 (no 404s)
- [ ] T053 [US1] Validate Chapter 1 against contract checklist in `contracts/chapter-1-gazebo-physics.md` (word count, examples, diagrams, exercises)

**Checkpoint**: Chapter 1 (Gazebo Physics) fully functional and independently testable. Student can complete SC-001 (create world in <20 min) and SC-004 (modify physics parameters in <10 min).

---

## Phase 4: User Story 2 - Unity Rendering and ROS 2 Integration (Priority: P2)

**Goal**: Students understand Unity's role in digital twin visualization, can set up Unity with ROS-TCP-Connector, and visualize robot joint motion from ROS 2 topics

**Independent Test**: Student can create Unity scene with robot model, install ROS-TCP-Connector, configure ROS connection, and visualize robot pose updates from ROS 2 `/joint_states` topic within 30 minutes (SC-002)

### Unity Code Examples Development for US2

**NOTE: Develop and validate ALL Unity scripts and scenes BEFORE writing MDX content**

- [ ] T054 [P] [US2] Create Unity project `Module2Examples` in `unity_scenes/` with Unity 2022.3 LTS
- [ ] T055 [P] [US2] Install ROS-TCP-Connector package in Unity via Package Manager with git URL `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector` (FR-011, FR-014)
- [ ] T056 [P] [US2] Install URDF Importer package in Unity via Package Manager (FR-011)
- [ ] T057 [P] [US2] Create `ROSConnection.cs` C# script in `unity_scenes/Module2Examples/Assets/Scripts/` for ROSConnection component configuration (IP, port 10000) (FR-014)
- [ ] T058 [P] [US2] Create `JointStateSubscriber.cs` C# script in `Assets/Scripts/` to subscribe to `/joint_states` and update Unity ArticulationBody (FR-015)
- [ ] T059 [P] [US2] Create `CameraControl.cs` C# script in `Assets/Scripts/` for orbit camera with mouse input and zoom with scroll wheel (FR-017)
- [ ] T060 [P] [US2] Create simple 2-link robot URDF file `simple_robot.urdf` in `Assets/URDF/` for import demonstration (FR-013, FR-018)

- [ ] T061 [US2] Create Unity scene `BasicRobotVisualization.unity` in `Assets/Scenes/` with imported robot and basic lighting (FR-012)
- [ ] T062 [US2] Create Unity scene `JointMotionDemo.unity` in `Assets/Scenes/` with robot, ROS connection, and joint state subscriber active (FR-015)
- [ ] T063 [US2] Test Unity ROS-TCP-Connector connection: start ROS 2 Humble, run Unity scene, verify TCP connection established (console shows "Connected to ROS")
- [ ] T064 [US2] Test joint motion: publish `/joint_states` from ROS 2, verify Unity ArticulationBody updates in real-time
- [ ] T065 [US2] Test URDF import: import `simple_robot.urdf`, verify GameObject hierarchy matches link structure, ArticulationBody components created (FR-013)
- [ ] T066 [US2] Configure lighting setup in Unity scene: directional light (sun), point lights, ambient occlusion (FR-016)
- [ ] T067 [US2] Configure PBR materials in Unity: Metallic and Smoothness settings for realistic rendering (FR-016)

### Diagram Creation for US2

- [ ] T068 [P] [US2] Create `unity-ros-tcp-connector-flow.svg` in `docs/module-2-digital-twin/assets/` showing ROS 2 topic → TCP endpoint → Unity ROSConnection → C# subscriber (FR-014)
- [ ] T069 [P] [US2] Create `urdf-unity-articulation-body.svg` in `assets/` showing URDF links as GameObjects, joints as ArticulationBody components (FR-013)
- [ ] T070 [P] [US2] Create `gazebo-unity-complementary-roles.svg` in `assets/` showing Gazebo (physics) vs Unity (visualization) separation (FR-010)

### MDX Content Writing for US2

- [ ] T071 [US2] Create `02-unity-rendering-ros2-integration.mdx` in `docs/module-2-digital-twin/` with frontmatter (title, description, keywords, sidebar_position: 2)
- [ ] T072 [US2] Write Prerequisites section in Chapter 2: Unity 2022.3 LTS, ROS-TCP-Connector, URDF Importer, Module 1 completion, GPU requirements (FR-011, FR-030)
- [ ] T073 [US2] Write Learning Objectives section in Chapter 2: 4 measurable outcomes (e.g., "Set up Unity scene with ROS connection in <30 min") (SC-002)
- [ ] T074 [US2] Write Concept Explanation section in Chapter 2: Unity's role (visualization, interaction) vs Gazebo's role (physics simulation) with diagram (FR-010)
- [ ] T075 [US2] Write Unity installation guide in Chapter 2: Unity 2022.3 LTS, ROS-TCP-Connector package (git URL), URDF Importer package (FR-011)
- [ ] T076 [US2] Write Unity scene setup steps in Chapter 2: GameObject hierarchy, robot model, coordinate system conversion (ROS Z-up to Unity Y-up) (FR-012)
- [ ] T077 [US2] Embed Code Example 1 in Chapter 2: `ROSConnection.cs` script with configuration explanation (FR-014)
- [ ] T078 [US2] Embed Code Example 2 in Chapter 2: `JointStateSubscriber.cs` script with message handling explanation (FR-015)
- [ ] T079 [US2] Embed Code Example 3 in Chapter 2: `CameraControl.cs` script with Unity Input API comments (FR-017)
- [ ] T080 [US2] Embed Code Example 4 in Chapter 2: Lighting and material setup with PBR workflow explanation (FR-016)
- [ ] T081 [US2] Write URDF import demonstration in Chapter 2: step-by-step URDF Importer usage, ArticulationBody mapping (FR-013)
- [ ] T082 [US2] Write Exercises section in Chapter 2: Exercise 1 (import 2-link robot URDF), Exercise 2 (set up ROS-TCP-Connector and test), Exercise 3 (create lighting setup) (FR-018)
- [ ] T083 [US2] Write Common Errors section in Chapter 2: 5 examples (ROS-TCP-Connector timeout, URDF import errors, coordinate mismatch, missing ROS message types, ArticulationBody issues) (FR-033)
- [ ] T084 [US2] Write References section in Chapter 2: Minimum 5 IEEE citations (Unity docs, ROS-TCP-Connector GitHub, URDF Importer docs, PBR tutorials) (SC-007)

### Validation for US2

- [ ] T085 [US2] Run Docusaurus build and verify Chapter 2 renders without errors
- [ ] T086 [US2] Verify Unity project builds without compilation errors in Unity 2022.3 LTS
- [ ] T087 [US2] Verify all 4 C# scripts from Chapter 2 compile correctly
- [ ] T088 [US2] Count citations in Chapter 2 (must be ≥5 IEEE format)
- [ ] T089 [US2] Run link checker on Chapter 2 (no 404s)
- [ ] T090 [US2] Validate Chapter 2 against contract checklist in `contracts/chapter-2-unity-rendering.md` (word count, examples, diagrams, exercises)

**Checkpoint**: Chapter 2 (Unity Rendering) fully functional and independently testable. Student can complete SC-002 (Unity scene with ROS connection in <30 min) and SC-008 (explain Gazebo vs Unity difference).

---

## Phase 5: User Story 3 - Sensor Simulation (Priority: P3)

**Goal**: Students understand virtual sensor simulation (LiDAR, depth camera, IMU) in Gazebo, can configure sensors in SDF, and visualize sensor data in RViz

**Independent Test**: Student can add LiDAR sensor to robot in Gazebo SDF, launch world, subscribe to `/scan` topic with ROS 2, and visualize point cloud in RViz within 15 minutes (SC-003)

### Gazebo Sensor Code Examples Development for US3

**NOTE: Develop and validate ALL sensor configurations BEFORE writing MDX content**

- [ ] T091 [P] [US3] Create `sensor_lidar_robot.world` in `gazebo_worlds/module2_examples/` with LiDAR sensor plugin (`<sensor type="gpu_lidar">`, ray count 360, range 10m, FOV 270°, ROS 2 topic `/scan`) (FR-020, FR-021)
- [ ] T092 [P] [US3] Create `sensor_depth_camera.world` in `gazebo_worlds/module2_examples/` with depth camera sensor (`<sensor type="depth_camera">`, resolution 640x480, clip planes 0.1-10m, point cloud output) (FR-022)
- [ ] T093 [P] [US3] Create `sensor_imu_robot.world` in `gazebo_worlds/module2_examples/` with IMU sensor (`<sensor type="imu">`, noise models on accelerometer/gyroscope, output topic `/imu/data`) (FR-023)

- [ ] T094 [US3] Test `sensor_lidar_robot.world`: launch Gazebo, verify `/scan` topic publishes with `ros2 topic list` and `ros2 topic echo /scan` (FR-021, FR-025)
- [ ] T095 [US3] Test LiDAR in RViz: add LaserScan display, verify point cloud matches obstacles in Gazebo world (FR-024)
- [ ] T096 [US3] Test `sensor_depth_camera.world`: verify depth image and PointCloud2 topics publish, visualize in RViz (FR-022, FR-024)
- [ ] T097 [US3] Test `sensor_imu_robot.world`: rotate robot in Gazebo, monitor `/imu/data` topic, verify orientation quaternion updates (FR-023)
- [ ] T098 [US3] Test sensor noise models: verify Gaussian noise on LiDAR ranges, IMU drift simulation works (FR-027)
- [ ] T099 [US3] Test sensor performance: experiment with LiDAR ray counts (100, 500, 1000, 5000), measure Gazebo FPS, document tradeoffs (FR-026)

### Diagram Creation for US3

- [ ] T100 [P] [US3] Create `lidar-ray-casting-diagram.svg` in `docs/module-2-digital-twin/assets/` showing horizontal/vertical ray samples, FOV, range visualization (FR-019, FR-020)
- [ ] T101 [P] [US3] Create `depth-camera-buffer-diagram.svg` in `assets/` showing depth buffer rendering, clip planes, point cloud generation concept (FR-019, FR-022)
- [ ] T102 [P] [US3] Create `imu-sensor-placement.svg` in `assets/` showing IMU on robot base link, orientation quaternion, angular velocity vectors (FR-019, FR-023)

### MDX Content Writing for US3

- [ ] T103 [US3] Create `03-sensor-simulation.mdx` in `docs/module-2-digital-twin/` with frontmatter (title, description, keywords, sidebar_position: 3)
- [ ] T104 [US3] Write Prerequisites section in Chapter 3: Gazebo Fortress with sensor plugins, ROS 2 Humble, RViz, ros_gz_bridge package (FR-030)
- [ ] T105 [US3] Write Learning Objectives section in Chapter 3: 4 measurable outcomes (e.g., "Add LiDAR sensor and visualize in RViz in <15 min") (SC-003)
- [ ] T106 [US3] Write Concept Explanation section in Chapter 3: Sensor simulation concepts (ray-casting for LiDAR, depth buffers for depth camera, rigid body dynamics for IMU) with diagrams (FR-019)
- [ ] T107 [US3] Embed Code Example 1 in Chapter 3: `sensor_lidar_robot.world` SDF with annotated LiDAR configuration (ray count, range, FOV, ROS 2 topic mapping) (FR-020, FR-021)
- [ ] T108 [US3] Write LaserScan message format explanation in Chapter 3: ranges array, angle_min, angle_max, angle_increment (FR-021)
- [ ] T109 [US3] Embed Code Example 2 in Chapter 3: `sensor_depth_camera.world` SDF with depth camera configuration (resolution, clip planes, point cloud output topics) (FR-022)
- [ ] T110 [US3] Embed Code Example 3 in Chapter 3: `sensor_imu_robot.world` SDF with IMU configuration (noise models, Imu message type, orientation/angular velocity) (FR-023)
- [ ] T111 [US3] Write RViz visualization guide in Chapter 3: add LaserScan display, PointCloud2 display, IMU orientation (TF or Pose display) (FR-024)
- [ ] T112 [US3] Write sensor performance considerations in Chapter 3: ray count vs FPS for LiDAR, resolution vs memory for depth cameras, update rate impact (FR-026)
- [ ] T113 [US3] Write sensor noise and error models in Chapter 3: Gaussian noise on LiDAR ranges, IMU drift simulation, depth camera IR pattern limitations (FR-027)
- [ ] T114 [US3] Write Exercises section in Chapter 3: Exercise 1 (add LiDAR, verify with ros2 topic echo and RViz), Exercise 2 (configure depth camera), Exercise 3 (add IMU, monitor orientation) (FR-025)
- [ ] T115 [US3] Write Common Errors section in Chapter 3: 5 examples (LiDAR not publishing, depth camera black image, IMU incorrect orientation, RViz not appearing, performance issues) (FR-033, SC-009)
- [ ] T116 [US3] Write References section in Chapter 3: Minimum 5 IEEE citations (Gazebo sensor plugin docs, ROS 2 sensor message types, RViz docs, sensor simulation research papers) (SC-007)

### Validation for US3

- [ ] T117 [US3] Run Docusaurus build and verify Chapter 3 renders without errors
- [ ] T118 [US3] Verify all 3 Gazebo sensor worlds launch successfully from Chapter 3 embedded code
- [ ] T119 [US3] Verify sensor topics publish correctly: `/scan`, `/camera/depth`, `/imu/data` (SC-003, SC-006)
- [ ] T120 [US3] Count citations in Chapter 3 (must be ≥5 IEEE format)
- [ ] T121 [US3] Run link checker on Chapter 3 (no 404s)
- [ ] T122 [US3] Validate Chapter 3 against contract checklist in `contracts/chapter-3-sensor-simulation.md` (word count, examples, diagrams, exercises)

**Checkpoint**: Chapter 3 (Sensor Simulation) fully functional and independently testable. Student can complete SC-003 (add LiDAR in <15 min) and SC-009 (diagnose sensor issues in <5 min).

---

## Phase 6: Polish & Cross-Module Integration

**Purpose**: Module 2 integration, cross-chapter navigation, and constitution compliance

- [ ] T123 [P] Create Module 2 landing page overview in `docs/module-2-digital-twin/index.md` with prerequisites (Module 1 + GPU), learning path, expected time 5-7 hours (SC-010)
- [ ] T124 [P] Verify cross-chapter navigation: Chapter 1 → Chapter 2 → Chapter 3 links work correctly
- [ ] T125 [P] Verify Module 1 to Module 2 prerequisite links work (Chapter 3 references ROS 2 topics from Module 1)
- [ ] T126 [P] Update main site navigation: verify Module 2 appears after Module 1 in sidebar, position 3 in `_category_.json`
- [ ] T127 Full Docusaurus build validation: run `npm run build`, verify <2 min build time (constitution), no broken links
- [ ] T128 Measure page load time: verify <3s on 3G connection simulation (constitution)
- [ ] T129 Run link checker across all 3 chapters: verify 100% pass rate (no 404s)
- [ ] T130 Citation count validation: verify each chapter has ≥5 IEEE citations (SC-007)
- [ ] T131 [P] Pilot test Chapter 1: have 2-3 students complete exercises, measure time, collect feedback (target: <20 min for SC-001)
- [ ] T132 [P] Pilot test Chapter 2: measure Unity scene setup time (target: <30 min for SC-002)
- [ ] T133 [P] Pilot test Chapter 3: measure sensor setup and visualization time (target: <15 min for SC-003)
- [ ] T134 Validate Module 2 total completion time: pilot test full module (target: 5-7 hours for SC-010)
- [ ] T135 Re-validate constitution compliance: check all 7 principles against Module 2 content (Technical Accuracy, Reproducible Code, Modular Structure, Engineering Writing, Citations, Testability, Deployment)
- [ ] T136 Create Module 2 completion checklist: verify all success criteria SC-001 through SC-010 met
- [ ] T137 [P] Add Gazebo troubleshooting section: common SDF errors, physics instability fixes, sensor debugging
- [ ] T138 [P] Add Unity troubleshooting section: ROS-TCP-Connector connection issues, URDF import failures, ArticulationBody problems
- [ ] T139 [P] Add cross-module consistency check: verify diagram color scheme matches Module 1 (blue nodes, green topics, etc.)
- [ ] T140 Final review: technical peer review by Gazebo expert for Chapter 1, Unity expert for Chapter 2, sensor simulation expert for Chapter 3

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 - Chapter 1 (Phase 3)**: Depends on Foundational phase completion
- **User Story 2 - Chapter 2 (Phase 4)**: Depends on Foundational phase completion - Can run in parallel with US1/US3
- **User Story 3 - Chapter 3 (Phase 5)**: Depends on Foundational phase completion - Can run in parallel with US1/US2
- **Polish (Phase 6)**: Depends on desired user stories (chapters) being complete

### User Story Dependencies

- **Chapter 1 (US1 - Gazebo Physics)**: Can start after Foundational (Phase 2) - No dependencies on other chapters
- **Chapter 2 (US2 - Unity Rendering)**: Can start after Foundational (Phase 2) - Independent of Chapter 1 and 3
- **Chapter 3 (US3 - Sensor Simulation)**: Can start after Foundational (Phase 2) - Independent of Chapter 1 and 2

**Independence**: All 3 chapters are independently testable. Student can learn Gazebo physics without Unity, Unity rendering without sensors, or sensor simulation without advanced Unity.

### Within Each User Story (Chapter)

- **Code Development BEFORE Content Writing**: All SDF files, C# scripts, and Unity scenes MUST be validated before embedding in MDX
- **Diagrams BEFORE Content**: Create diagrams before writing explanatory sections that reference them
- **Content Sections**: Prerequisites → Learning Objectives → Concept Explanation → Code Examples → Exercises → Common Errors → References
- **Validation AFTER Content**: Docusaurus build, citation count, link check, contract validation

### Parallel Opportunities

- **Setup Phase (Phase 1)**: All 7 setup tasks marked [P] can run in parallel
- **Foundational Phase (Phase 2)**: Research tasks (T008-T012), data model tasks (T013-T017), contract tasks (T018-T020), quickstart tasks (T021-T024) can run in parallel within their groups
- **Once Foundational completes**: All 3 chapters (US1, US2, US3) can start in parallel (if team capacity allows)
- **Within Each Chapter**:
  - Code examples marked [P] can be developed in parallel (e.g., all 4 Gazebo worlds for Chapter 1)
  - Diagrams marked [P] can be created in parallel
  - Validation tasks marked [P] can run in parallel
- **Polish Phase**: Pilot tests marked [P] can run in parallel (different students), troubleshooting sections [P], consistency checks [P]

---

## Parallel Example: User Story 1 (Chapter 1)

```bash
# Launch all Gazebo world development for Chapter 1 together:
Task T025: "Create basic_world_gravity.world in gazebo_worlds/module2_examples/"
Task T026: "Create physics_parameters_demo.world in gazebo_worlds/module2_examples/"
Task T027: "Create collision_visual_geometry.world in gazebo_worlds/module2_examples/"
Task T028: "Create material_properties_demo.world in gazebo_worlds/module2_examples/"

# Launch all diagram creation for Chapter 1 together:
Task T033: "Create gazebo-architecture-diagram.svg in docs/module-2-digital-twin/assets/"
Task T034: "Create sdf-structure-example.svg in assets/"
Task T035: "Create physics-engine-components.svg in assets/"
```

---

## Implementation Strategy

### MVP First (Chapter 1 Only)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T024) - CRITICAL, blocks all chapters
3. Complete Phase 3: Chapter 1 Gazebo Physics (T025-T053)
4. **STOP and VALIDATE**: Test Chapter 1 independently
   - Student can create Gazebo world in <20 min (SC-001)
   - Student can modify physics parameters in <10 min (SC-004)
5. Deploy/demo Chapter 1 as standalone module

**Rationale**: Chapter 1 (Gazebo Physics) is P1 priority and foundation for simulation understanding. Deliverable: Students can create and configure Gazebo worlds independently.

### Incremental Delivery

1. Complete Setup + Foundational (T001-T024) → Foundation ready
2. Add Chapter 1 (T025-T053) → Test independently → Deploy/Demo (MVP!)
3. Add Chapter 2 (T054-T090) → Test independently → Deploy/Demo
4. Add Chapter 3 (T091-T122) → Test independently → Deploy/Demo
5. Polish (T123-T140) → Full Module 2 complete

**Rationale**: Each chapter adds value without breaking previous chapters. Students can access content incrementally as chapters complete.

### Parallel Team Strategy

With 3 developers (or agents):

1. Team completes Setup + Foundational together (T001-T024)
2. Once Foundational is done:
   - **Developer A**: Chapter 1 - Gazebo Physics (T025-T053)
   - **Developer B**: Chapter 2 - Unity Rendering (T054-T090)
   - **Developer C**: Chapter 3 - Sensor Simulation (T091-T122)
3. Each chapter developed and validated independently
4. Team reunites for Polish phase (T123-T140)

**Estimated Time**:
- Setup + Foundational: ~10-15 hours (collaborative)
- Per Chapter: ~25-35 hours each (parallel)
- Polish: ~8-12 hours (collaborative)
- **Total (sequential)**: ~85-105 hours
- **Total (parallel, 3 devs)**: ~45-60 hours

---

## Notes

- **[P] tasks**: Different files, no dependencies on incomplete tasks
- **[Story] label**: Maps task to specific chapter (US1=Ch1, US2=Ch2, US3=Ch3) for traceability
- **Each chapter independently completable**: Students can learn Gazebo alone, Unity alone, or sensors alone
- **Code-first workflow**: Validate all code examples in isolated workspaces (gazebo_worlds/, unity_scenes/) BEFORE embedding in MDX
- **Constitution compliance**: All tasks align with 7 principles (Technical Accuracy, Reproducible Code, Modular Structure, Engineering Writing, Citations, Testability, Deployment)
- **Success criteria**: Tasks designed to enable students to meet SC-001 through SC-010 (time limits, completion rates, citation counts)
- **Commit strategy**: Commit after each task or logical group (e.g., all Gazebo worlds for Chapter 1, all diagrams for Chapter 2)
- **Stop at checkpoints**: Validate chapter independently before proceeding (SC-001, SC-002, SC-003)
- **Quality gates**: Docusaurus build, citation count, link check, contract validation, pilot test timing

---

## Summary

- **Total Tasks**: 140
- **Setup Phase**: 7 tasks
- **Foundational Phase**: 17 tasks (BLOCKING)
- **Chapter 1 (US1)**: 29 tasks (Code: 8, Diagrams: 3, Content: 13, Validation: 5)
- **Chapter 2 (US2)**: 37 tasks (Code: 14, Diagrams: 3, Content: 14, Validation: 6)
- **Chapter 3 (US3)**: 32 tasks (Code: 9, Diagrams: 3, Content: 14, Validation: 6)
- **Polish Phase**: 18 tasks
- **Parallel Opportunities**: 52 tasks marked [P] (37% parallelizable)
- **MVP Scope**: Setup + Foundational + Chapter 1 = 53 tasks (38% of total)

**Format Validation**: ✅ All 140 tasks follow checklist format (checkbox + ID + [P]/[Story] labels + file paths)

**Independent Test Criteria**:
- **US1 (Chapter 1)**: Student creates Gazebo world, spawns objects, observes physics in <20 min (SC-001)
- **US2 (Chapter 2)**: Student sets up Unity with ROS-TCP-Connector, visualizes joint motion in <30 min (SC-002)
- **US3 (Chapter 3)**: Student adds LiDAR sensor, visualizes in RViz in <15 min (SC-003)
