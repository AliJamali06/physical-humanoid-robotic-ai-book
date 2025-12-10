# Research: Module 3 – The AI-Robot Brain (NVIDIA Isaac)

**Feature**: `003-isaac-perception-nav`
**Date**: 2025-12-09
**Phase**: Phase 0 (Outline & Research)

## Purpose

This document resolves all "NEEDS CLARIFICATION" items from Technical Context and researches best practices for educational content on NVIDIA Isaac Sim, Isaac ROS VSLAM, and Nav2 navigation. Research findings inform Phase 1 design decisions (data models, contracts, quickstart).

## Research Tasks

### 1. Isaac Sim Synthetic Data Generation Workflow

**Question**: What is the authoritative workflow for generating synthetic training data using Isaac Sim Replicator API?

**Findings**:
- **Official Documentation**: [NVIDIA Isaac Sim Documentation - Replicator](https://docs.nvidia.com/isaac/doc/isaac_sim/replicator.html)
- **Workflow**: Scene setup (load USD assets) → Configure Replicator Writer (output format: ROS bag, NumPy, COCO) → Define randomizers (lighting, textures, object poses) → Run capture loop → Export data
- **Code Pattern**:
  ```python
  import omni.replicator.core as rep

  # Define camera and output writer
  camera = rep.create.camera(position=(5, 5, 5))
  render_product = rep.create.render_product(camera, (1024, 1024))
  writer = rep.WriterRegistry.get("BasicWriter")
  writer.initialize(output_dir="synthetic_data", rgb=True, semantic_segmentation=True)

  # Define randomization function
  def randomize_scene():
      rep.randomizer.lighting(intensity_range=(0.5, 2.0))
      rep.randomizer.texture(materials_path="/Materials")
      return rep.distribution.uniform(0.0, 1.0)

  # Register and trigger
  rep.randomizer.register(randomize_scene)
  with rep.trigger.on_frame(num_frames=100):
      rep.randomizer.randomize_scene()
  ```
- **Key Concepts**: Domain randomization (lighting variation, texture swapping, procedural placement), ground truth labels (pixel-perfect segmentation masks), output formats (ROS 2 bags for direct pipeline integration)
- **Educational Approach**: Provide conceptual workflow with code snippet (as above) for reference; clarify that execution requires Isaac Sim installation (optional hands-on)

**Decision**: Use official NVIDIA Isaac Sim Replicator documentation as primary source. Include code snippet with comments explaining each stage (scene → randomization → capture → export). Emphasize value proposition: cost savings, data diversity, perfect labels.

---

### 2. Visual SLAM Pipeline Components and Isaac ROS Integration

**Question**: What are the essential stages of Visual SLAM, and how does Isaac ROS implement GPU-accelerated VSLAM?

**Findings**:
- **VSLAM Fundamentals** (research papers: ORB-SLAM2 [Mur-Artal2017], LSD-SLAM [Engel2014]):
  1. **Feature Extraction**: Detect keypoints in images (ORB, FAST, SIFT descriptors)
  2. **Feature Matching**: Track features across consecutive frames
  3. **Motion Estimation**: Compute camera pose from matched features (visual odometry)
  4. **Mapping**: Build 3D map of landmark points
  5. **Loop Closure**: Detect revisited locations to correct drift
  6. **Map Optimization**: Bundle adjustment (refine poses and landmarks globally)

- **Isaac ROS VSLAM** ([Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/concepts/visual_slam/index.html)):
  - GPU-accelerated implementation using CUDA for feature extraction and matching
  - ROS 2 node: `visual_slam_node`
  - Input topics: `/camera/image_raw` (RGB), `/camera/camera_info` (intrinsics)
  - Output topics: `/visual_slam/tracking/odometry` (Odometry), `/visual_slam/tracking/vo_pose` (PoseStamped), `/visual_slam/vis/slam_path` (Path)
  - Performance: ~100 Hz odometry update rate on NVIDIA Jetson AGX Orin (vs ~10-30 Hz CPU-based SLAM)
  - Configuration: YAML parameters for map resolution, feature detector type, loop closure frequency

- **Integration with Isaac Sim**:
  - Launch Isaac Sim scene with camera-equipped robot
  - Publish camera images to `/camera/image_raw` topic (via Isaac ROS bridge)
  - Run `visual_slam_node` to consume images and publish odometry
  - Visualize trajectory in RViz using `/visual_slam/vis/slam_path` topic

**Decision**: Explain VSLAM pipeline with clear diagram (6 stages as listed above). Describe Isaac ROS architecture with ROS 2 topic flow diagram. Compare CPU vs GPU performance using published benchmarks. Provide example YAML configuration for `visual_slam_node` with comments.

**Alternatives Considered**:
- **ORB-SLAM3** (CPU-based, widely used): Rejected for this module because Module 3 focuses on Isaac ecosystem and GPU acceleration benefits
- **RTAB-Map** (ROS 1/2 compatible): Mentioned as alternative in "Further Reading" section, but Isaac ROS is primary focus

---

### 3. Nav2 Architecture and ROS 2 Navigation Stack

**Question**: How does Nav2 (ROS 2 Navigation Stack) integrate VSLAM localization with path planning and obstacle avoidance?

**Findings**:
- **Nav2 Architecture** ([Nav2 Documentation](https://navigation.ros.org/)):
  - **Localization**: VSLAM (Isaac ROS) or AMCL provides robot pose (`/tf` transform: `map` → `odom` → `base_link`)
  - **Global Planner**: Finds optimal path from start to goal on cost map (Dijkstra, A*, NavFn, Smac Planner)
  - **Local Planner**: Generates short-term trajectories for dynamic obstacle avoidance (DWB, TEB, MPPI controllers)
  - **Controller**: Converts trajectory to velocity commands `/cmd_vel` (linear and angular velocities)
  - **Cost Maps**: 2D occupancy grids with layers (static map, obstacle layer from sensors, inflation layer for safety margin)

- **Cost Map Representation**:
  - Grid cells: `0` = free space, `100` = occupied, `255` = unknown
  - Inflation radius: padding around obstacles (configurable via `inflation_radius` parameter)
  - Update frequency: global cost map updates at ~1 Hz, local cost map at ~5-10 Hz

- **ROS 2 Topics**:
  - Input: `/goal_pose` (PoseStamped), `/map` (OccupancyGrid), `/scan` or `/camera/depth` (sensor data)
  - Output: `/cmd_vel` (Twist), `/plan` (Path for global plan), `/local_plan` (Path for local trajectory)

- **Humanoid-Specific Considerations**:
  - **Footprint**: Rectangular or polygon (vs circular for wheeled robots) to match humanoid body dimensions
  - **Footstep Planning**: Not directly supported by Nav2 (requires custom planner or integration with humanoid-specific stack like BipedalLocomotion or IHMC)
  - **Gait Patterns**: Nav2 assumes continuous motion; bipedal walking requires discrete foot placements and balance constraints
  - **Educational Approach**: Introduce Nav2 as foundational navigation stack, explain how humanoid robots require additional planning layers (conceptual overview, not full implementation)

**Decision**: Describe Nav2 architecture with diagram showing localization → global planner → local planner → controller flow. Explain cost map representation with visual example (grid with occupied/free/inflated cells). Provide YAML config snippets for global planner (NavFn or Smac) and local planner (DWB) parameters. Introduce humanoid navigation conceptually (footstep planning, stability constraints) without requiring hands-on implementation.

**Alternatives Considered**:
- **move_base (ROS 1)**: Rejected because Module 1 established ROS 2 Humble as the standard
- **Custom A\* implementation**: Rejected because Nav2 provides battle-tested planners; educational focus is on understanding architecture, not reimplementing algorithms

---

### 4. Diagram Creation Tools and Best Practices

**Question**: What tools and styles should be used for consistent, high-quality diagrams in educational content?

**Findings**:
- **Tools**:
  - **Mermaid.js**: Supported natively in Docusaurus MDX, version-controlled (text-based), easy to update
  - **Draw.io/Diagrams.net**: SVG export, more flexible layouts, requires external tool
  - **Decision**: Use Mermaid.js for simple flow diagrams (ROS 2 topic flows, VSLAM pipeline), SVG exports from Draw.io for complex architectural diagrams (Isaac Sim architecture, Nav2 cost map visualization)

- **Style Guide** (from FR-033):
  - Color scheme: Perception nodes (purple), Planning nodes (orange), Sensor data (green), Commands (red)
  - Format: SVG (scalable, small file size)
  - Annotations: Clear labels for nodes, arrows, and data flows

- **Example Diagrams Needed**:
  1. Isaac Sim Architecture (components: simulation engine, rendering, sensor simulation, ROS 2 bridge)
  2. VSLAM Pipeline (6 stages: feature extraction → matching → odometry → mapping → loop closure → optimization)
  3. Nav2 Cost Map Flow (localization → global planner → local planner → controller → `/cmd_vel`)
  4. ROS 2 Topic Graph (Isaac Sim → `/camera/image_raw` → `visual_slam_node` → `/visual_slam/tracking/odometry` → Nav2)

**Decision**: Use Mermaid.js for ROS 2 topic flow diagrams (embedded in MDX). Create SVG diagrams in Draw.io for Isaac Sim architecture and Nav2 cost map visualization. Follow color scheme: purple (perception), orange (planning), green (sensors), red (commands). Store SVGs in `static/img/module-3/`.

---

### 5. Code Example Validation Workflow

**Question**: How should code examples be validated before embedding in educational content?

**Findings**:
- **Best Practices** (from constitution Principle II):
  - Test code in isolated environment before including in documentation
  - Include complete context: imports, dependencies, environment setup
  - Provide expected output or behavior description

- **Validation Workflow**:
  1. **Python Scripts** (Isaac Replicator API):
     - Create standalone `.py` file with all imports
     - Test execution in Isaac Sim Python environment (if available) or document expected behavior
     - Include docstring with prerequisites ("Requires Isaac Sim 2023.1.1 or later")
     - Store in `static/code/module-3/isaac_replicator_example.py`

  2. **YAML Configs** (ROS 2 Isaac ROS VSLAM, Nav2 parameters):
     - Validate YAML syntax using `yamllint` or Python `yaml.safe_load()`
     - Compare against official examples from Isaac ROS GitHub or Nav2 documentation
     - Include inline comments explaining each parameter
     - Store in `static/code/module-3/vslam_config.yaml` and `nav2_params.yaml`

  3. **Bash Commands** (ROS 2 CLI):
     - Test commands in ROS 2 Humble environment (if available) or verify against official ROS 2 documentation
     - Provide expected output in comments or following code block
     - Example: `ros2 topic echo /visual_slam/tracking/odometry` with sample Odometry message output

**Decision**: Store all code examples as standalone files in `static/code/module-3/`. Validate Python scripts against Isaac Sim API documentation (execution optional if no GPU available). Validate YAML configs using `yamllint` and comparison with official examples. Test bash commands against ROS 2 Humble documentation. Embed validated code in MDX using Docusaurus `CodeBlock` component with language-specific syntax highlighting.

---

### 6. Citation and Reference Management

**Question**: What sources should be cited for Isaac Sim, Isaac ROS, Nav2, and SLAM research?

**Findings**:
- **Official Documentation** (primary sources):
  1. [NVIDIA Isaac Sim Documentation](https://docs.nvidia.com/isaac/doc/isaac_sim/index.html)
  2. [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
  3. [Nav2 Documentation](https://navigation.ros.org/)
  4. [ROS 2 Documentation](https://docs.ros.org/en/humble/)

- **Research Papers** (SLAM fundamentals, minimum 5 per chapter per FR-034):
  - **SLAM Survey**: Cadena, C., et al. "Past, Present, and Future of Simultaneous Localization and Mapping: Toward the Robust-Perception Age." IEEE Transactions on Robotics, 2016.
  - **ORB-SLAM2**: Mur-Artal, R., and Tardós, J. D. "ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras." IEEE Transactions on Robotics, 2017.
  - **Visual Odometry**: Scaramuzza, D., and Fraundorfer, F. "Visual Odometry: Part I - The First 30 Years and Fundamentals." IEEE Robotics & Automation Magazine, 2011.
  - **Nav2 Original Paper**: Macenski, S., et al. "Robot Operating System 2: Design, Architecture, and Uses In The Wild." Science Robotics, 2022.
  - **Dynamic Window Approach**: Fox, D., Burgard, W., and Thrun, S. "The Dynamic Window Approach to Collision Avoidance." IEEE Robotics & Automation Magazine, 1997.

- **Citation Format** (IEEE, per FR-034):
  - Inline: `[Mur-Artal2017]` or `[NVIDIA2023]`
  - Bibliography: `[1] R. Mur-Artal and J. D. Tardós, "ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras," IEEE Trans. Robot., vol. 33, no. 5, pp. 1255-1262, Oct. 2017.`

**Decision**: Create bibliography section at end of each chapter with minimum 5 IEEE-format citations (mix of official docs and research papers). Use inline citation format `[AuthorYear]` for research papers and `[NVIDIA2023]`, `[ROS2Docs]`, `[Nav2Docs]` for official documentation. Store full bibliography in Docusaurus markdown using `<References>` section.

---

## Summary of Research Decisions

| Research Area | Decision | Rationale |
|---------------|----------|-----------|
| **Isaac Sim Workflow** | Use official NVIDIA Replicator API documentation and code example with domain randomization | Authoritative source, aligns with Principle I (Technical Accuracy), provides reproducible workflow |
| **VSLAM Pipeline** | Explain 6-stage pipeline (feature extraction → matching → odometry → mapping → loop closure → optimization) with Isaac ROS GPU acceleration | Aligns with educational goal (conceptual understanding), Isaac ROS as primary focus, CPU alternatives mentioned in "Further Reading" |
| **Nav2 Architecture** | Describe localization → global planner → local planner → controller flow with cost map representation | ROS 2 standard navigation stack, aligns with Module 1-2 prerequisites, provides foundation for humanoid navigation concepts |
| **Diagram Tools** | Mermaid.js for ROS 2 topic flows, SVG (Draw.io) for architectural diagrams, color-coded (purple/orange/green/red) | Version-controlled (Mermaid.js), high-quality visuals (SVG), consistent style (FR-033) |
| **Code Validation** | Standalone files in `static/code/module-3/`, validated against official docs, syntax checked | Principle II (Reproducible Code), prevents untested examples in documentation |
| **Citations** | Minimum 5 IEEE-format citations per chapter (NVIDIA docs, Isaac ROS, Nav2, SLAM research papers) | Principle V (Citation Standards), provides authoritative backing for all technical claims |

## Next Steps

**Phase 1 (Design & Contracts)**:
1. Generate `data-model.md` with entities from spec.md (Isaac Sim Scene, Synthetic Data Sample, VSLAM Map, Odometry Estimate, Cost Map, Navigation Goal, Global Path, Local Trajectory)
2. Create `contracts/` directory with API schemas (if applicable for educational content; likely N/A for this feature since it's pure documentation)
3. Generate `quickstart.md` with "Getting Started with Module 3" guide (prerequisites, chapter overview, learning objectives)
4. Update agent context (add NVIDIA Isaac, Isaac ROS, Nav2 technologies to Claude/agent memory)

**Phase 2 (/sp.tasks)**:
- Generate dependency-ordered task breakdown for implementing Module 3 content (MDX chapters, diagrams, code examples, validation)
