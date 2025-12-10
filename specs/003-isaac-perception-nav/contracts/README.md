# Contracts: Module 3 – The AI-Robot Brain (NVIDIA Isaac)

**Feature**: `003-isaac-perception-nav`
**Date**: 2025-12-09
**Phase**: Phase 1 (Design & Contracts)

## Purpose

This directory contains interface contracts for Module 3. Since Module 3 is educational documentation (not a running system with HTTP APIs), the primary "contracts" are **ROS 2 message schemas** that define data structures exchanged between Isaac Sim, Isaac ROS VSLAM, and Nav2.

## ROS 2 Message Contracts

### 1. Camera Image (Isaac Sim → Isaac ROS VSLAM)

**Topic**: `/camera/image_raw`
**Message Type**: `sensor_msgs/Image`

```yaml
# sensor_msgs/Image
header:
  stamp:          # ROS 2 Time (seconds + nanoseconds)
    sec: 0
    nanosec: 0
  frame_id: "camera_link"  # Camera frame name

height: 1024      # Image height in pixels
width: 1024       # Image width in pixels
encoding: "rgb8"  # Pixel encoding (rgb8 = 8-bit RGB)
is_bigendian: 0   # Endianness (0 = little-endian)
step: 3072        # Row length in bytes (width * 3 for rgb8)
data: []          # Image data (height * step bytes)
```

**Contract**:
- `height` and `width` must match camera configuration in Isaac Sim
- `encoding` must be `rgb8` (Isaac ROS VSLAM expects 8-bit RGB)
- `frame_id` must match camera link in robot URDF/USD
- Published at 30-60 Hz for real-time VSLAM

---

### 2. Camera Intrinsics (Isaac Sim → Isaac ROS VSLAM)

**Topic**: `/camera/camera_info`
**Message Type**: `sensor_msgs/CameraInfo`

```yaml
# sensor_msgs/CameraInfo
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "camera_link"

height: 1024
width: 1024

distortion_model: "plumb_bob"  # Lens distortion model
D: [0.0, 0.0, 0.0, 0.0, 0.0]   # Distortion coefficients (k1, k2, t1, t2, k3)

K: [800.0, 0.0, 512.0,          # Intrinsic matrix (3x3)
    0.0, 800.0, 512.0,
    0.0, 0.0, 1.0]

R: [1.0, 0.0, 0.0,              # Rectification matrix (3x3, identity for monocular)
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0]

P: [800.0, 0.0, 512.0, 0.0,     # Projection matrix (3x4)
    0.0, 800.0, 512.0, 0.0,
    0.0, 0.0, 1.0, 0.0]

binning_x: 0
binning_y: 0

roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: false
```

**Contract**:
- `K` (intrinsic matrix) must match camera FOV and resolution (focal length = K[0], principal point = K[2], K[5])
- `D` distortion coefficients must match camera lens model (often [0, 0, 0, 0, 0] for ideal pinhole camera in simulation)
- `header.stamp` must match corresponding `/camera/image_raw` message
- Published at same rate as `/camera/image_raw` (synchronized)

---

### 3. Visual Odometry Output (Isaac ROS VSLAM → Nav2)

**Topic**: `/visual_slam/tracking/odometry`
**Message Type**: `nav_msgs/Odometry`

```yaml
# nav_msgs/Odometry
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "map"  # Or "odom" for local tracking

child_frame_id: "base_link"  # Robot base frame

pose:
  pose:
    position:
      x: 0.0  # meters
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0  # Quaternion
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.01, 0, 0, 0, 0, 0,   # 6x6 covariance (position xyz, orientation rpy)
               0, 0.01, 0, 0, 0, 0,
               0, 0, 0.01, 0, 0, 0,
               0, 0, 0, 0.01, 0, 0,
               0, 0, 0, 0, 0.01, 0,
               0, 0, 0, 0, 0, 0.01]

twist:
  twist:
    linear:
      x: 0.0  # Linear velocity (m/s)
      y: 0.0
      z: 0.0
    angular:
      x: 0.0  # Angular velocity (rad/s)
      y: 0.0
      z: 0.0
  covariance: [0.01, 0, 0, 0, 0, 0,
               0, 0.01, 0, 0, 0, 0,
               0, 0, 0.01, 0, 0, 0,
               0, 0, 0, 0.01, 0, 0,
               0, 0, 0, 0, 0.01, 0,
               0, 0, 0, 0, 0, 0.01]
```

**Contract**:
- `orientation` quaternion must be normalized (w² + x² + y² + z² = 1.0)
- `covariance` must be symmetric positive semi-definite (represents uncertainty)
- `frame_id` typically `map` (for global SLAM) or `odom` (for local tracking)
- Published at 30-100 Hz (Isaac ROS GPU acceleration enables high-rate odometry)

---

### 4. Navigation Goal (User/RViz → Nav2)

**Topic**: `/goal_pose`
**Message Type**: `geometry_msgs/PoseStamped`

```yaml
# geometry_msgs/PoseStamped
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "map"  # Goal must be in global map frame

pose:
  position:
    x: 5.0  # Target position in meters
    y: 3.0
    z: 0.0  # Typically 0 for 2D navigation
  orientation:
    x: 0.0  # Target orientation (quaternion)
    y: 0.0
    z: 0.0
    w: 1.0  # Facing +x axis (0 degrees yaw)
```

**Contract**:
- `frame_id` must be `map` (Nav2 expects goals in global coordinate system)
- `position` must be within known map bounds (not in occupied or unknown space)
- `orientation` quaternion must be normalized
- Single message submission triggers NavigateToPose action

---

### 5. Global Path (Nav2 Global Planner → Local Planner)

**Topic**: `/plan`
**Message Type**: `nav_msgs/Path`

```yaml
# nav_msgs/Path
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "map"

poses:
  - header:
      stamp: {sec: 0, nanosec: 0}
      frame_id: "map"
    pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - header:
      stamp: {sec: 0, nanosec: 0}
      frame_id: "map"
    pose:
      position: {x: 1.0, y: 0.5, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  # ... more waypoints ...
  - header:
      stamp: {sec: 0, nanosec: 0}
      frame_id: "map"
    pose:
      position: {x: 5.0, y: 3.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

**Contract**:
- First pose must be robot's current position (or very close)
- Last pose must match navigation goal
- All poses must be collision-free on cost map
- Published at ~1 Hz (updated when cost map changes or goal changes)

---

### 6. Velocity Commands (Nav2 Local Planner → Robot)

**Topic**: `/cmd_vel`
**Message Type**: `geometry_msgs/Twist`

```yaml
# geometry_msgs/Twist
linear:
  x: 0.5   # Linear velocity (m/s, forward/backward)
  y: 0.0   # Lateral velocity (m/s, for holonomic robots, 0 for humanoid)
  z: 0.0   # Vertical velocity (m/s, typically 0 for ground robots)

angular:
  x: 0.0   # Roll rate (rad/s, typically 0 for ground robots)
  y: 0.0   # Pitch rate (rad/s, typically 0 for ground robots)
  z: 0.2   # Yaw rate (rad/s, rotation about vertical axis)
```

**Contract**:
- `linear.x` must be within robot's max linear velocity (e.g., -1.0 to 1.0 m/s)
- `angular.z` must be within robot's max angular velocity (e.g., -1.0 to 1.0 rad/s)
- `linear.y` typically 0 for non-holonomic humanoid (bipedal robots cannot strafe)
- Published at 10-20 Hz by Nav2 controller
- Robot must convert to joint commands (gait controller for humanoid)

---

### 7. Cost Map (Nav2 → Global/Local Planners)

**Topic**: `/global_costmap/costmap` or `/local_costmap/costmap`
**Message Type**: `nav_msgs/OccupancyGrid`

```yaml
# nav_msgs/OccupancyGrid
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "map"  # or "odom" for local cost map

info:
  map_load_time: {sec: 0, nanosec: 0}
  resolution: 0.05  # meters per cell (5 cm resolution)
  width: 200        # Number of cells in x direction
  height: 200       # Number of cells in y direction
  origin:
    position: {x: -5.0, y: -5.0, z: 0.0}  # Bottom-left corner in frame_id
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}

data: [0, 0, 0, 100, 100, 100, ...]  # Flattened 2D array (row-major order)
                                      # 0 = free, 100 = occupied, 255 = unknown
```

**Contract**:
- `resolution` must be > 0.0 (typically 0.01 to 0.1 meters)
- `data` length must equal `width * height`
- Cell values: `0` (free space), `1-99` (low cost), `100` (occupied), `255` (unknown)
- Global cost map updated at ~1 Hz, local cost map at ~5-10 Hz
- Inflation layer adds padding around obstacles (configurable `inflation_radius`)

---

## Configuration Contracts (YAML Parameters)

### Isaac ROS VSLAM Configuration

**File**: `vslam_config.yaml`

```yaml
visual_slam_node:
  ros__parameters:
    # Camera parameters
    camera_frame_id: "camera_link"
    map_frame_id: "map"
    odom_frame_id: "odom"
    base_frame_id: "base_link"

    # VSLAM algorithm parameters
    num_cameras: 1  # Monocular VSLAM
    feature_tracker_type: "ORB"  # ORB, FAST, or SIFT
    num_features: 500  # Features to track per frame

    # Loop closure
    enable_loop_closure: true
    loop_closure_frequency: 1.0  # Hz

    # Performance
    gpu_id: 0  # CUDA device ID
    publish_tf: true  # Publish map->odom transform
    publish_odom_tf: true  # Publish odom->base_link transform
```

**Contract**:
- `camera_frame_id` must match camera link in robot URDF
- `num_features` trade-off: higher = more accurate but slower (typical: 200-1000)
- `enable_loop_closure` must be `true` for long-term SLAM (corrects drift)
- `gpu_id` must match available NVIDIA GPU (0 for single-GPU systems)

---

### Nav2 Global Planner Configuration

**File**: `nav2_params.yaml` (partial)

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"  # Dijkstra-based planner
      tolerance: 0.5  # Acceptable distance to goal (meters)
      use_astar: true  # Enable A* (faster than Dijkstra)
      allow_unknown: false  # Reject paths through unknown space
```

**Contract**:
- `tolerance` must be > 0.0 (typically 0.1-0.5 meters)
- `use_astar: true` enables heuristic-guided search (faster, still optimal)
- `allow_unknown: false` prevents robot from entering unexplored areas

---

### Nav2 Local Planner Configuration

**File**: `nav2_params.yaml` (partial)

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"  # Dynamic Window Approach
      max_vel_x: 0.5  # Max forward velocity (m/s)
      min_vel_x: -0.2  # Max backward velocity (m/s)
      max_vel_theta: 1.0  # Max rotation rate (rad/s)
      acc_lim_x: 2.0  # Linear acceleration limit (m/s²)
      acc_lim_theta: 3.0  # Angular acceleration limit (rad/s²)

      # Trajectory scoring weights
      path_distance_bias: 32.0  # Prefer trajectories close to global path
      goal_distance_bias: 20.0  # Prefer trajectories toward goal
      occdist_scale: 0.02  # Penalize trajectories near obstacles
```

**Contract**:
- `max_vel_x`, `max_vel_theta` must match robot's kinematic limits
- Acceleration limits prevent jerky motion (important for humanoid stability)
- Scoring weights trade-off: high `path_distance_bias` = stick to global path, high `occdist_scale` = avoid obstacles aggressively

---

## Validation Criteria

**For Educational Content** (Module 3):
- Students will review these message schemas in chapter content (not implement ROS 2 nodes)
- Code examples will show how to `ros2 topic echo` these messages to understand structure
- YAML configs will be provided as reference (students can modify parameters conceptually)

**For Code Examples**:
- All ROS 2 message YAML examples must validate against official `ros2 interface show <msg_type>`
- YAML config files must pass `yamllint` syntax check
- Example values must be realistic (e.g., velocities within typical humanoid robot limits)

---

## References

- [ROS 2 sensor_msgs](https://docs.ros2.org/latest/api/sensor_msgs/)
- [ROS 2 nav_msgs](https://docs.ros2.org/latest/api/nav_msgs/)
- [ROS 2 geometry_msgs](https://docs.ros2.org/latest/api/geometry_msgs/)
- [Isaac ROS Visual SLAM Configuration](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#quickstart)
- [Nav2 Configuration Guide](https://navigation.ros.org/configuration/index.html)
