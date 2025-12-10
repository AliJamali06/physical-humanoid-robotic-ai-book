# Feature Specification: Module 2 – The Digital Twin (Gazebo & Unity)

**Feature Branch**: `002-digital-twin-gazebo-unity`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Module 2 – The Digital Twin (Gazebo & Unity) - Target audience: Robotics students learning simulation and environment building. Focus: Physics simulation in Gazebo, high-fidelity interaction in Unity, and sensor simulation (LiDAR, Depth, IMU). Success criteria: 2–3 chapters explaining Gazebo physics, Unity rendering, and sensor simulation; Students can run a Gazebo world with gravity/collision; Students understand how LiDAR, depth cameras, and IMUs are simulated. Constraints: Format: Markdown (Docusaurus-ready), Include simple runnable Gazebo examples, Keep explanations accurate and beginner-friendly. Not building: Full humanoid navigation stack, Isaac integration (Module 3), Advanced Unity scripting or game development."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Gazebo Physics Simulation Fundamentals (Priority: P1)

A robotics student needs to understand how physics engines simulate real-world robot behavior in Gazebo, including gravity, collision detection, friction, and basic world creation. They learn by creating simple worlds with obstacles and observing physics interactions.

**Why this priority**: Physics simulation is the foundation for all digital twin work. Without understanding how Gazebo simulates gravity, collision, and dynamics, students cannot progress to more complex sensor simulation or multi-robot scenarios. This is the minimum viable knowledge for simulation-based robotics.

**Independent Test**: Student can create a Gazebo world file (.world) with ground plane, gravity enabled, and simple geometric objects (boxes, spheres), launch the world, spawn a robot model, and observe realistic physics behavior (falling, collision response) using Gazebo GUI within 20 minutes.

**Acceptance Scenarios**:

1. **Given** student has Gazebo Fortress installed, **When** they create a basic .world file with ground and gravity, **Then** they can launch it with `gz sim world.sdf` and see the 3D environment
2. **Given** world is running, **When** student spawns a box object above the ground, **Then** the box falls and collides with the ground plane realistically
3. **Given** student wants to test collisions, **When** they create multiple objects with collision geometry, **Then** objects interact physically (bounce, slide) based on material properties
4. **Given** student modifies friction parameters in SDF, **When** they respawn objects, **Then** sliding behavior changes visibly (e.g., ice vs rubber)
5. **Given** student wants to understand world structure, **When** they examine the .world/.sdf file, **Then** they can identify physics engine settings (gravity vector, time step), model definitions, and plugin configurations

---

### User Story 2 - Unity High-Fidelity Rendering and Interaction (Priority: P2)

A student needs to understand how Unity provides photorealistic rendering and interactive visualization for robotics simulation, complementing Gazebo's physics with visual quality and user interaction capabilities. They learn to set up Unity scenes with ROS 2 integration for visualization.

**Why this priority**: Unity excels at rendering and user interaction but lacks robust physics. Combined with Gazebo (physics) via ROS 2 bridge, it creates complete digital twin. This builds on Gazebo knowledge by adding visualization layer. Can be tested independently by creating static Unity scenes.

**Independent Test**: Student can create a Unity scene with a simple robot model (imported URDF), set up basic lighting and camera, install ROS-TCP-Connector package, and visualize robot pose updates from ROS 2 topics within 30 minutes (no physics, just visualization).

**Acceptance Scenarios**:

1. **Given** student has Unity 2022.3 LTS and ROS-TCP-Connector installed, **When** they create new Unity project, **Then** they can import robotics assets and configure ROS connection settings
2. **Given** Unity scene is prepared, **When** student imports a simple URDF-based robot model, **Then** the robot appears in the scene hierarchy with proper joint structure
3. **Given** student wants realistic visuals, **When** they add lighting (directional, ambient) and configure materials (PBR shaders), **Then** robot and environment render with shadows and reflections
4. **Given** student runs ROS 2 publisher sending joint states, **When** Unity receives messages via ROS-TCP-Connector, **Then** robot joints update in real-time to match published positions
5. **Given** student wants camera viewpoints, **When** they configure multiple camera angles (orbit, first-person, top-down), **Then** they can switch views to observe robot from different perspectives

---

### User Story 3 - Sensor Simulation (LiDAR, Depth Camera, IMU) (Priority: P3)

A student needs to understand how virtual sensors (LiDAR, depth cameras, IMUs) are simulated in Gazebo to generate realistic sensor data for robot perception algorithms. They learn sensor models, data formats, and how to visualize sensor output.

**Why this priority**: Sensor simulation is critical for testing perception pipelines without physical hardware. Builds on Gazebo physics (P1) and benefits from Unity visualization (P2) but can be tested independently using Gazebo's built-in sensor plugins and RViz.

**Independent Test**: Student can add a LiDAR sensor plugin to a robot in Gazebo SDF, launch the world, subscribe to `/scan` topic with ROS 2, visualize point cloud in RViz, and verify that obstacles appear correctly in sensor data within 15 minutes.

**Acceptance Scenarios**:

1. **Given** student has a Gazebo world with robot model, **When** they add `<sensor type="gpu_lidar">` to robot SDF with ray count and range parameters, **Then** sensor publishes LaserScan messages to ROS 2 topic
2. **Given** LiDAR is publishing, **When** student runs `ros2 topic echo /scan`, **Then** they see range data arrays corresponding to detected obstacles at correct distances
3. **Given** student wants depth camera, **When** they configure `<sensor type="depth_camera">` with resolution and FOV, **Then** sensor publishes depth images and point clouds to ROS 2 topics
4. **Given** depth camera is active, **When** student visualizes in RViz, **Then** they see 3D point cloud representation of environment matching camera view
5. **Given** student needs IMU data, **When** they add `<sensor type="imu">` to robot base link, **Then** sensor publishes orientation (quaternion) and angular velocity to `/imu` topic
6. **Given** student rotates robot in Gazebo, **When** they monitor IMU topic, **Then** orientation data updates to reflect robot's 3D orientation changes

---

### Edge Cases

- **What happens when** Gazebo physics time step is too large? (Answer: Simulation becomes unstable, objects penetrate each other - students test with different <max_step_size> values and observe behavior)
- **How does system handle** Unity ROS connection loss during runtime? (Answer: Unity stops receiving updates, robot freezes in last known pose - students test by stopping ROS 2 publisher and observing)
- **What happens when** LiDAR ray count is set extremely high (e.g., 10,000 rays)? (Answer: Simulation slows down significantly - students understand performance vs accuracy tradeoff)
- **How does depth camera handle** reflective or transparent surfaces in Gazebo? (Answer: May produce incorrect depth values or noise - students learn sensor limitations and when to use depth vs RGB cameras)
- **What happens when** IMU sensor is placed far from robot center of mass? (Answer: Introduces additional angular acceleration noise - students learn proper sensor placement for humanoid robots)

## Requirements *(mandatory)*

### Functional Requirements

#### Chapter 1: Gazebo Physics and World Building
- **FR-001**: Chapter MUST explain Gazebo architecture (physics engine, rendering, sensor simulation) with clear diagram showing components
- **FR-002**: Chapter MUST provide step-by-step tutorial for creating basic .world/.sdf file with ground plane, gravity settings, and light sources
- **FR-003**: World examples MUST include physics parameters (gravity vector, time step, solver type) with explanations of each parameter's impact
- **FR-004**: Chapter MUST demonstrate collision geometry vs visual geometry with annotated SDF showing `<collision>` and `<visual>` tags
- **FR-005**: Chapter MUST explain material properties (friction coefficients: mu, mu2; restitution for bounciness) with runnable examples showing different materials (ice, rubber, wood)
- **FR-006**: Chapter MUST include Gazebo plugin basics: how to load world plugins (e.g., physics parameters plugin) with example SDF snippets
- **FR-007**: Chapter MUST provide commands for launching Gazebo: `gz sim <world_file>.sdf`, `gz model --spawn-file <model>.sdf`, `gz topic -l` for topic inspection
- **FR-008**: Chapter MUST include practical exercise where student creates world with ramp, box, and sphere, then adjusts friction to observe sliding behavior
- **FR-009**: Chapter MUST explain how to debug physics issues using Gazebo GUI (wireframe view for collisions, contact visualization, center of mass display)

#### Chapter 2: Unity Rendering and ROS 2 Integration
- **FR-010**: Chapter MUST explain Unity's role in digital twin (visualization, user interaction) vs Gazebo's role (physics simulation)
- **FR-011**: Chapter MUST provide installation guide for Unity 2022.3 LTS, ROS-TCP-Connector package (via Package Manager with git URL), and URDF Importer package
- **FR-012**: Chapter MUST include step-by-step Unity scene setup: create GameObject hierarchy, add robot model, configure transforms matching ROS coordinate system (Y-up in Unity, Z-up in ROS)
- **FR-013**: Unity examples MUST demonstrate importing URDF into Unity (using URDF Importer package) with explanation of how links become GameObjects and joints become ArticulationBody components
- **FR-014**: Chapter MUST explain ROS-TCP-Connector configuration: ROSConnection.cs component, setting ROS IP address, port (default 10000), and testing connection
- **FR-015**: Chapter MUST provide C# script example for subscribing to ROS 2 `/joint_states` topic and updating Unity robot joint angles in real-time
- **FR-016**: Chapter MUST demonstrate lighting setup (directional light for sun, point lights for lamps, ambient occlusion) and material configuration (Metallic, Smoothness for realistic rendering)
- **FR-017**: Chapter MUST include camera control script (orbit camera with mouse input, zoom with scroll wheel) with code comments explaining Unity's transform and Input API
- **FR-018**: Chapter MUST provide practical exercise where student creates Unity scene, imports simple 2-link robot URDF, and visualizes joint motion from ROS 2 test publisher

#### Chapter 3: Sensor Simulation in Gazebo
- **FR-019**: Chapter MUST explain sensor simulation concepts: ray-casting for LiDAR, depth buffers for depth cameras, rigid body dynamics for IMU
- **FR-020**: Chapter MUST provide annotated SDF showing LiDAR sensor configuration: `<sensor type="gpu_lidar">`, `<ray><scan><horizontal>` (samples, resolution, min/max angle), `<range>` (min, max), update rate
- **FR-021**: LiDAR examples MUST include ROS 2 topic mapping (`<ros><topic>/scan</topic></ros>`) and explanation of LaserScan message format (ranges array, angle_min, angle_max, angle_increment)
- **FR-022**: Chapter MUST demonstrate depth camera sensor: `<sensor type="depth_camera">`, `<camera><image>` (width, height), `<clip>` (near, far), and how it publishes both depth images and point clouds
- **FR-023**: Chapter MUST explain IMU sensor configuration: `<sensor type="imu">`, `<imu><noise>` for realistic noise models (Gaussian noise on accelerometer, gyroscope), and output topics (/imu/data for Imu message type)
- **FR-024**: Chapter MUST include RViz visualization guide: how to add LaserScan display, PointCloud2 display, and IMU orientation (using TF or Pose display)
- **FR-025**: Chapter MUST provide practical exercise where student adds LiDAR to robot, spawns obstacles, and verifies ranges match obstacle distances using `ros2 topic echo /scan` and RViz
- **FR-026**: Chapter MUST explain sensor performance considerations: ray count vs FPS for LiDAR, resolution vs memory for depth cameras, update rate impact on CPU usage
- **FR-027**: Chapter MUST include sensor noise and error models: how to add Gaussian noise to LiDAR ranges, IMU drift simulation, depth camera IR pattern limitations

#### Cross-Chapter Requirements
- **FR-028**: All Gazebo examples MUST be tested on Gazebo Fortress (or Gazebo Garden/Harmonic with version noted) running on Ubuntu 22.04
- **FR-029**: Unity examples MUST be tested on Unity 2022.3 LTS with ROS-TCP-Connector compatible with ROS 2 Humble
- **FR-030**: Each chapter MUST include "Prerequisites" section listing required software (Gazebo version, Unity version, ROS 2 Humble, RViz) and system requirements (GPU for rendering, RAM for Unity)
- **FR-031**: Each chapter MUST include "Learning Objectives" section with 3-5 measurable outcomes (e.g., "Create Gazebo world with custom physics parameters in 15 minutes")
- **FR-032**: All code (SDF, C# scripts) MUST include inline comments explaining key lines and configuration options
- **FR-033**: Chapters MUST include "Common Errors" section showing typical mistakes (e.g., wrong SDF syntax, Unity-ROS connection failures, missing ROS message types) with solutions
- **FR-034**: Each chapter MUST cite official documentation using IEEE format: Gazebo documentation, Unity documentation, ROS-TCP-Connector GitHub, sensor plugin references
- **FR-035**: Markdown content MUST include code blocks with syntax highlighting: ` ```xml ` for SDF, ` ```csharp ` for Unity C# scripts, ` ```bash ` for terminal commands

### Key Entities

- **Gazebo World**: Simulation environment container. Attributes: physics engine settings (gravity, time step, solver), models (ground, obstacles, robots), plugins (for ROS 2 bridge, custom physics), lighting (sun, ambient).
- **SDF (Simulation Description Format)**: XML-based format for defining Gazebo worlds and models. Attributes: `<world>` (environment), `<model>` (objects/robots), `<link>` (rigid body), `<joint>` (connections), `<sensor>` (virtual sensors), `<plugin>` (custom behaviors).
- **Physics Engine**: Simulates dynamics and collisions. Attributes: engine type (ODE, Bullet, DART), max_step_size (integration time step, e.g., 0.001s), iterations (solver accuracy), gravity vector (default: [0, 0, -9.81] m/s²).
- **Unity Scene**: 3D visualization environment. Attributes: GameObjects (scene graph nodes), Transforms (position, rotation, scale), Camera (viewpoint), Lights (directional, point, spot), Materials (shaders for appearance).
- **ROS-TCP-Connector**: Unity package for ROS 2 communication. Attributes: ROSConnection component, TCP endpoint configuration (IP, port), message serialization/deserialization, topic publishers/subscribers in Unity.
- **URDF in Unity**: Robot model representation. Attributes: imported as GameObject hierarchy, links as ArticulationBody (Unity physics joints), visual meshes, coordinate system conversion (ROS Z-up to Unity Y-up).
- **LiDAR Sensor**: Simulated laser scanner. Attributes: ray count (horizontal/vertical samples), range (min/max distance in meters), FOV (field of view angle), update rate (Hz), noise model (Gaussian on ranges).
- **Depth Camera**: Simulated RGB-D sensor. Attributes: resolution (width x height pixels), FOV (horizontal/vertical angles), clip planes (near/far in meters), output topics (depth image, point cloud), format (float32 for depth values).
- **IMU Sensor**: Simulated inertial measurement unit. Attributes: orientation (quaternion), angular velocity (rad/s), linear acceleration (m/s²), noise parameters (accel/gyro bias, white noise), attached link (usually robot base).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Student can create and launch a Gazebo world with custom gravity and collision objects within 20 minutes after reading Chapter 1, verified by running `gz sim` and observing physics behavior
- **SC-002**: Student can successfully configure Unity scene with ROS-TCP-Connector and visualize robot joint motion from ROS 2 topic within 30 minutes after reading Chapter 2, verified by real-time Unity updates
- **SC-003**: Student can add a LiDAR sensor to robot in Gazebo, launch world, and visualize scan data in RViz within 15 minutes after reading Chapter 3
- **SC-004**: Student can identify and modify physics parameters (friction, restitution) in SDF file and observe behavioral changes within 10 minutes after completing Chapter 1 exercises
- **SC-005**: 85% of students successfully complete all chapter exercises on first attempt (measured by exercise submission success rate or self-assessment checklist)
- **SC-006**: All Gazebo world examples launch without errors on Gazebo Fortress (Ubuntu 22.04) and all Unity examples run on Unity 2022.3 LTS, verified by CI test pipeline or manual testing
- **SC-007**: Each chapter contains minimum 5 IEEE-format citations to official Gazebo documentation, Unity documentation, ROS-TCP-Connector GitHub, or robotics sensor simulation research papers
- **SC-008**: Student can explain the difference between physics simulation (Gazebo) and rendering (Unity) in their own words after completing Chapters 1-2 (measured by comprehension quiz: 75% pass rate)
- **SC-009**: Student can diagnose common sensor simulation issues (e.g., LiDAR not publishing, depth camera showing black image) using ROS 2 introspection tools (`ros2 topic list`, `ros2 topic echo`) within 5 minutes
- **SC-010**: Module content completion time averages 5-7 hours for beginner student familiar with ROS 2 basics (Module 1 prerequisite), measured by pilot reader feedback and self-reported time
