# ROS 2 Topic Flow: Isaac Sim → VSLAM → Nav2

This diagram shows the complete data flow from Isaac Sim synthetic sensors through Visual SLAM localization to Nav2 autonomous navigation.

```mermaid
graph LR
    %% Define nodes with color coding
    IsaacSim[Isaac Sim<br/>Simulation Engine]:::simulation
    CameraNode[Camera Sensor]:::sensor
    ImageTopic[/camera/image_raw]:::data
    CameraInfo[/camera/camera_info]:::data

    VSLAMNode[visual_slam_node]:::perception
    OdomTopic[/visual_slam/tracking/odometry]:::data
    PoseTopic[/visual_slam/tracking/vo_pose]:::data
    PathTopic[/visual_slam/vis/slam_path]:::data
    TFMap[TF: map → odom]:::data

    Nav2Stack[Nav2 Navigation Stack]:::planning
    GoalPose[/goal_pose]:::command
    GlobalPlan[/plan]:::data
    LocalPlan[/local_plan]:::data
    CmdVel[/cmd_vel]:::command

    Robot[Humanoid Robot<br/>Base Controller]:::robot

    %% Isaac Sim → Camera Data Flow
    IsaacSim --> CameraNode
    CameraNode -->|RGB Images| ImageTopic
    CameraNode -->|Intrinsics| CameraInfo

    %% Camera Data → VSLAM
    ImageTopic --> VSLAMNode
    CameraInfo --> VSLAMNode

    %% VSLAM → Localization Outputs
    VSLAMNode -->|Position/Velocity| OdomTopic
    VSLAMNode -->|Camera Pose| PoseTopic
    VSLAMNode -->|Trajectory| PathTopic
    VSLAMNode -->|Transform| TFMap

    %% Localization → Nav2
    TFMap --> Nav2Stack
    OdomTopic --> Nav2Stack

    %% Nav2 Planning
    GoalPose -->|User Input| Nav2Stack
    Nav2Stack -->|Global Path| GlobalPlan
    Nav2Stack -->|Local Trajectory| LocalPlan
    Nav2Stack -->|Velocity Commands| CmdVel

    %% Nav2 → Robot Control
    CmdVel --> Robot

    %% Color scheme definitions (FR-033)
    classDef simulation fill:#e1f5ff,stroke:#0084c7,stroke-width:2px
    classDef sensor fill:#c8e6c9,stroke:#4caf50,stroke-width:2px,color:#000
    classDef data fill:#fff9c4,stroke:#fbc02d,stroke-width:2px,color:#000
    classDef perception fill:#e1bee7,stroke:#9c27b0,stroke-width:3px,color:#000
    classDef planning fill:#ffe0b2,stroke:#ff9800,stroke-width:3px,color:#000
    classDef command fill:#ffccbc,stroke:#ff5722,stroke-width:2px,color:#000
    classDef robot fill:#cfd8dc,stroke:#607d8b,stroke-width:2px,color:#000
```

## Color Legend

- **Blue (Simulation)**: Isaac Sim components
- **Green (Sensors)**: Sensor data sources (cameras, depth, etc.)
- **Yellow (Data)**: ROS 2 topics and TF transforms
- **Purple (Perception)**: Perception processing nodes (VSLAM)
- **Orange (Planning)**: Planning and navigation nodes (Nav2)
- **Red (Commands)**: Control commands sent to robot
- **Gray (Robot)**: Physical robot hardware or simulated base controller

## Key Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | RGB camera images from Isaac Sim |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsic parameters |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | Robot position and velocity estimate |
| `/visual_slam/tracking/vo_pose` | `geometry_msgs/PoseStamped` | Camera pose in map frame |
| `/visual_slam/vis/slam_path` | `nav_msgs/Path` | VSLAM trajectory visualization |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal from user/planner |
| `/plan` | `nav_msgs/Path` | Global path from Nav2 planner |
| `/local_plan` | `nav_msgs/Path` | Local trajectory from dynamic planner |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (linear, angular) |

## TF Frames

The VSLAM node publishes the transform tree:

```
map → odom → base_link → camera_link
```

- **map**: World-fixed reference frame (VSLAM global map)
- **odom**: Odometry frame (tracks robot motion)
- **base_link**: Robot base coordinate frame
- **camera_link**: Camera sensor frame

This transform tree allows Nav2 to plan paths in the `map` frame while the robot executes them using real-time odometry updates.
