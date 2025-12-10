# Module 3 Diagram Requirements

This document specifies all diagrams needed for Module 3. Diagrams should be created using Draw.io (diagrams.net) and exported as SVG files following the style guide in `docs/module-3/diagram-style-guide.md`.

## Color Scheme (from FR-033)

- **Perception nodes**: Purple `#9c27b0`
- **Planning nodes**: Orange `#ff9800`
- **Sensor data**: Green `#4caf50`
- **Commands**: Red `#ff5722`
- **Simulation**: Blue `#0084c7`
- **Data topics**: Yellow `#fbc02d`
- **Robot hardware**: Gray `#607d8b`

---

## Chapter 1: Isaac Sim Synthetic Data Generation

### 1. isaac-sim-architecture.svg

**Purpose**: Show Isaac Sim's component architecture

**Layout**: Hierarchical (top-to-bottom)

**Components**:
```
┌─────────────────────────────────────────────────┐
│   Simulation Engine (USD Scene Management)     │ (Blue #0084c7)
│   - Hierarchical scene graph (prims)           │
│   - Non-destructive editing (layers)           │
└─────────────────────────────────────────────────┘
                    ↓
┌──────────────────────────┐  ┌──────────────────────────┐
│  RTX Ray-Traced          │  │  PhysX Physics Engine    │ (Blue)
│  Rendering               │  │  - Rigid body dynamics   │
│  - Path tracing          │  │  - Collisions            │
│  - PBR materials         │  │  - Articulated bodies    │
└──────────────────────────┘  └──────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│   Sensor Simulation                             │ (Green #4caf50)
│   RGB | Depth | Semantic Seg | Instance Seg     │
│   LiDAR | IMU | Contact Sensors                 │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│   ROS 2 Bridge                                  │ (Yellow #fbc02d)
│   /camera/image_raw → sensor_msgs/Image         │
│   /camera/depth → sensor_msgs/Image             │
│   /camera/camera_info → sensor_msgs/CameraInfo  │
└─────────────────────────────────────────────────┘
```

**Arrows**: Solid black lines with labels ("feeds into", "publishes to")

**File**: `static/img/module-3/isaac-sim-architecture.svg`

---

### 2. isaac-sim-pipeline.svg

**Purpose**: Show synthetic data generation workflow

**Layout**: Left-to-right pipeline

**Stages**:
```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│ 1. Scene    │ →  │ 2. Domain   │ →  │ 3. Sensor   │ →  │ 4. Data     │ →  │ 5. Export   │
│    Setup    │    │ Randomize   │    │  Capture    │    │  Annotate   │    │  to Disk    │
│             │    │             │    │             │    │             │    │             │
│ Load USD    │    │ - Lighting  │    │ - RGB       │    │ - Bbox      │    │ - COCO      │
│ assets,     │    │ - Textures  │    │ - Depth     │    │ - Segm.     │    │ - ROS bag   │
│ position    │    │ - Placement │    │ - Segm.     │    │ - Pose      │    │ - NumPy     │
│ objects     │    │ - Camera    │    │             │    │             │    │             │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
```

**Colors**:
- Stage 1: Blue (simulation)
- Stage 2: Orange (planning/randomization)
- Stage 3: Green (sensors)
- Stage 4: Purple (perception/processing)
- Stage 5: Yellow (data output)

**File**: `static/img/module-3/isaac-sim-pipeline.svg`

---

### 3. isaac-ui-overview.png (Screenshot)

**Purpose**: Show Isaac Sim user interface

**Content**: Screenshot of Isaac Sim with:
- Left panel: Stage hierarchy showing `World/Table/Cup/Camera`
- Center: 3D viewport showing table with cup
- Right panel: Property panel with Transform settings visible

**Annotations**: Add numbered labels:
1. "Stage (Scene Hierarchy)"
2. "Viewport (3D View)"
3. "Property Panel"
4. "Simulation Controls (Play/Pause/Stop)"

**File**: `static/img/module-3/screenshots/isaac-ui-overview.png`

**Note**: This requires actual Isaac Sim screenshot. If unavailable, use placeholder with text description.

---

## Chapter 2: Visual SLAM with Isaac ROS

### 4. vslam-pipeline.svg

**Purpose**: Show 6-stage VSLAM pipeline

**Layout**: Left-to-right flow

**Stages**:
```
┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│  1. Feature  │ → │  2. Feature  │ → │  3. Motion   │ → │  4. Mapping  │ → │  5. Loop     │ → │  6. Map      │
│  Extraction  │   │   Matching   │   │  Estimation  │   │              │   │  Closure     │   │ Optimization │
│              │   │              │   │              │   │              │   │              │   │              │
│ Detect ORB/  │   │ Track feat.  │   │ Compute      │   │ Build 3D     │   │ Detect       │   │ Bundle       │
│ FAST keypts  │   │ across       │   │ camera pose  │   │ landmark     │   │ revisited    │   │ adjustment   │
│              │   │ frames       │   │ (VO)         │   │ map          │   │ locations    │   │              │
└──────────────┘   └──────────────┘   └──────────────┘   └──────────────┘   └──────────────┘   └──────────────┘
```

**Colors**: All stages purple (perception processing)

**File**: `static/img/module-3/vslam-pipeline.svg`

---

### 5. isaac-ros-architecture.svg

**Purpose**: Show Isaac ROS VSLAM node architecture with ROS 2 topics

**Layout**: Center node with input/output topics

**Structure**:
```
          ┌─────────────────────┐
          │ /camera/image_raw   │ (Green - sensor)
          │ sensor_msgs/Image   │
          └─────────────────────┘
                    ↓
          ┌─────────────────────┐
          │ /camera/camera_info │ (Green)
          │ CameraInfo          │
          └─────────────────────┘
                    ↓
    ┌───────────────────────────────────────┐
    │      visual_slam_node                 │ (Purple - perception)
    │                                       │
    │  - GPU-accelerated feature extraction │
    │  - CUDA-based feature matching        │
    │  - Loop closure detection             │
    │  - Map optimization                   │
    └───────────────────────────────────────┘
         ↓                ↓                ↓
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│ /visual_slam/│  │ /visual_slam/│  │  TF: map →   │
│ tracking/    │  │ tracking/    │  │  odom        │
│ odometry     │  │ vo_pose      │  │              │
│ (Yellow)     │  │ (Yellow)     │  │ (Yellow)     │
└──────────────┘  └──────────────┘  └──────────────┘
```

**File**: `static/img/module-3/isaac-ros-architecture.svg`

---

### 6. vslam-tf-tree.svg

**Purpose**: Show TF transform tree for VSLAM

**Layout**: Hierarchical tree (left-to-right)

**Structure**:
```
map → odom → base_link → camera_link
             ↓
          ↓
      left_wheel  right_wheel
```

**Labels**:
- `map`: World-fixed reference frame (VSLAM global map)
- `odom`: Odometry frame (tracks robot motion, may drift)
- `base_link`: Robot base coordinate frame
- `camera_link`: Camera sensor frame

**File**: `static/img/module-3/vslam-tf-tree.svg`

---

### 7. rviz-vslam-trajectory.png (Screenshot)

**Purpose**: Show RViz visualization of VSLAM trajectory

**Content**: Screenshot of RViz showing:
- Grid floor (map frame)
- Red/green/blue axes (coordinate frames)
- Yellow path (VSLAM trajectory `/visual_slam/vis/slam_path`)
- Point cloud or camera frustum

**Annotations**: Label key elements (trajectory, axes, map)

**File**: `static/img/module-3/screenshots/rviz-vslam-trajectory.png`

**Note**: Requires RViz screenshot. If unavailable, use placeholder diagram.

---

## Chapter 3: Nav2 Humanoid Navigation

### 8. nav2-architecture.svg

**Purpose**: Show Nav2 navigation pipeline

**Layout**: Left-to-right flow with feedback loop

**Structure**:
```
┌──────────────┐
│ Localization │ (Purple - perception)
│ (VSLAM/AMCL) │
└──────────────┘
       ↓
       ↓ Pose estimate
       ↓
┌──────────────────────────────────────────────┐
│           Nav2 Navigation Stack              │ (Orange - planning)
│  ┌──────────────┐  ┌──────────────┐         │
│  │ Global       │→ │ Local        │→ /cmd_vel│
│  │ Planner      │  │ Planner      │  (Red)   │
│  │ (A*, Dijkstra)  │ (DWB, TEB)   │         │
│  └──────────────┘  └──────────────┘         │
│         ↑                 ↑                  │
│    Cost Map          Cost Map               │
│    (Static)          (Dynamic)              │
└──────────────────────────────────────────────┘
       ↓
       ↓ Velocity commands
       ↓
┌──────────────┐
│ Robot Base   │ (Gray - hardware)
│ Controller   │
└──────────────┘
```

**File**: `static/img/module-3/nav2-architecture.svg`

---

### 9. nav2-costmap.svg

**Purpose**: Visualize cost map layers (static, obstacle, inflation)

**Layout**: 2D grid with legend

**Content**:
- Grid (10x10 cells)
- Black cells: occupied (static obstacles)
- Dark gray cells: dynamic obstacles
- Light gray gradient: inflation layer (safety margin around obstacles)
- White cells: free space
- Yellow cells: unknown space

**Legend**:
```
■ Occupied (cost=100)
░ Inflated (cost=50-99)
□ Free (cost=0)
▒ Unknown (cost=255)
```

**File**: `static/img/module-3/nav2-costmap.svg`

---

### 10. nav2-planning-comparison.svg

**Purpose**: Compare global planner (A*) vs local planner (DWB)

**Layout**: Side-by-side comparison

**Content**:

**Left panel: Global Planner**
- Grid with start (green) and goal (red) points
- Blue path from start to goal (A* path)
- Static obstacles (black squares)
- Label: "Global Planner (A*, NavFn)"
- Label: "Plans path on static cost map"

**Right panel: Local Planner**
- Zoomed-in view of robot
- Green trajectory arc (local trajectory)
- Red dynamic obstacle (detected by sensors)
- Label: "Local Planner (DWB)"
- Label: "Generates short-term trajectory for obstacle avoidance"

**File**: `static/img/module-3/nav2-planning-comparison.svg`

---

### 11. costmap-inflation-layer.png (Screenshot)

**Purpose**: Show RViz cost map visualization

**Content**: Screenshot of RViz showing:
- Grid cost map with obstacles
- Inflation radius around obstacles (gradient from dark to light)
- Robot footprint (rectangular for humanoid)

**Annotations**: Label inflation radius, obstacles, free space

**File**: `static/img/module-3/screenshots/costmap-inflation-layer.png`

**Note**: Requires RViz screenshot. If unavailable, create SVG diagram instead.

---

## Implementation Priority

### High Priority (Required for MVP)
1. `isaac-sim-architecture.svg` (Chapter 1)
2. `isaac-sim-pipeline.svg` (Chapter 1)
3. `vslam-pipeline.svg` (Chapter 2)
4. `isaac-ros-architecture.svg` (Chapter 2)
5. `nav2-architecture.svg` (Chapter 3)
6. `nav2-costmap.svg` (Chapter 3)

### Medium Priority (Enhances Understanding)
7. `vslam-tf-tree.svg` (Chapter 2)
8. `nav2-planning-comparison.svg` (Chapter 3)

### Low Priority (Optional Screenshots)
9. `isaac-ui-overview.png` (Chapter 1 screenshot)
10. `rviz-vslam-trajectory.png` (Chapter 2 screenshot)
11. `costmap-inflation-layer.png` (Chapter 3 screenshot)

---

## Creation Workflow

For each SVG diagram:

1. Open Draw.io (https://app.diagrams.net/)
2. Create new blank diagram
3. Use rounded rectangles for components (border-radius: 10px)
4. Apply color scheme from style guide
5. Add clear labels (14pt font, bold for titles)
6. Export as SVG: File → Export as → SVG
   - Uncheck "Transparent Background"
   - Check "Include a copy of my diagram"
7. Save to `static/img/module-3/`
8. Verify SVG displays correctly in MDX

---

## Notes

- All diagrams follow the color scheme in `docs/module-3/diagram-style-guide.md`
- Screenshots require Isaac Sim and RViz2 installations
- If screenshots are unavailable, create equivalent SVG diagrams with annotations
- Mermaid.js diagrams are already embedded in MDX chapter files (no separate SVG needed)
