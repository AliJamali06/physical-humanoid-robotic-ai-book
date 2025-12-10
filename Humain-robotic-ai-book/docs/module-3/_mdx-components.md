# MDX Component Guide for Module 3

This guide documents the reusable MDX components and code block configurations used across Module 3 chapters.

## Code Block Syntax Highlighting

### Python Code Blocks

Use triple backticks with `python` language identifier:

\`\`\`python
import omni.replicator.core as rep

# Example: Isaac Sim Replicator API
camera = rep.create.camera(position=(5, 5, 5))
render_product = rep.create.render_product(camera, (1024, 1024))
\`\`\`

**Renders as**:
```python
import omni.replicator.core as rep

# Example: Isaac Sim Replicator API
camera = rep.create.camera(position=(5, 5, 5))
render_product = rep.create.render_product(camera, (1024, 1024))
```

### YAML Configuration Blocks

Use triple backticks with `yaml` language identifier:

\`\`\`yaml
# Isaac ROS VSLAM Configuration
visual_slam:
  ros__parameters:
    num_cameras: 1
    camera_frame_id: "camera_link"
    map_frame: "map"
    odom_frame: "odom"
\`\`\`

**Renders as**:
```yaml
# Isaac ROS VSLAM Configuration
visual_slam:
  ros__parameters:
    num_cameras: 1
    camera_frame_id: "camera_link"
    map_frame: "map"
    odom_frame: "odom"
```

### Bash Command Blocks

Use triple backticks with `bash` language identifier:

\`\`\`bash
# Launch Isaac ROS VSLAM node
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Echo odometry topic
ros2 topic echo /visual_slam/tracking/odometry
\`\`\`

**Renders as**:
```bash
# Launch Isaac ROS VSLAM node
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Echo odometry topic
ros2 topic echo /visual_slam/tracking/odometry
```

### ROS 2 Message Definitions

Use triple backticks with `text` or `msg` language identifier:

\`\`\`msg
# nav_msgs/Odometry
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
\`\`\`

**Renders as**:
```text
# nav_msgs/Odometry
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

## Importing External Code Files

### Python Script from static/code/

\`\`\`mdx
import CodeBlock from '@theme/CodeBlock';
import IsaacReplicatorExample from '!!raw-loader!@site/static/code/module-3/isaac_replicator_example.py';

<CodeBlock language="python" title="static/code/module-3/isaac_replicator_example.py">
  {IsaacReplicatorExample}
</CodeBlock>
\`\`\`

### YAML Configuration from static/code/

\`\`\`mdx
import CodeBlock from '@theme/CodeBlock';
import VSLAMConfig from '!!raw-loader!@site/static/code/module-3/vslam_config.yaml';

<CodeBlock language="yaml" title="vslam_config.yaml" showLineNumbers>
  {VSLAMConfig}
</CodeBlock>
\`\`\`

## Admonitions (Callout Boxes)

### Tip (Learning Objectives)

\`\`\`mdx
:::tip Learning Objective
After completing this section, you should be able to:
- Explain the Isaac Sim Replicator API workflow (scene → randomization → capture)
- Identify three domain randomization techniques (lighting, textures, placement)
- Describe the value of synthetic data for robot perception training
:::
\`\`\`

**Renders as**:
:::tip Learning Objective
After completing this section, you should be able to:
- Explain the Isaac Sim Replicator API workflow (scene → randomization → capture)
- Identify three domain randomization techniques (lighting, textures, placement)
- Describe the value of synthetic data for robot perception training
:::

### Note (Prerequisites)

\`\`\`mdx
:::note Prerequisites
Before starting this chapter, you should have:
- Completed Module 1 (ROS 2 fundamentals) and Module 2 (Gazebo simulation)
- Basic understanding of Python programming
- Familiarity with 3D graphics concepts (cameras, coordinate frames, rendering)
:::
\`\`\`

**Renders as**:
:::note Prerequisites
Before starting this chapter, you should have:
- Completed Module 1 (ROS 2 fundamentals) and Module 2 (Gazebo simulation)
- Basic understanding of Python programming
- Familiarity with 3D graphics concepts (cameras, coordinate frames, rendering)
:::

### Warning (Common Pitfalls)

\`\`\`mdx
:::warning Common Misconception
**Myth**: "VSLAM provides perfect localization without drift."

**Reality**: All SLAM systems accumulate drift over time. Loop closure and map optimization reduce drift, but VSLAM is not a replacement for GPS in outdoor environments or wheel odometry for short-term motion estimation.
:::
\`\`\`

**Renders as**:
:::warning Common Misconception
**Myth**: "VSLAM provides perfect localization without drift."

**Reality**: All SLAM systems accumulate drift over time. Loop closure and map optimization reduce drift, but VSLAM is not a replacement for GPS in outdoor environments or wheel odometry for short-term motion estimation.
:::

### Caution (Hardware Requirements)

\`\`\`mdx
:::caution Hardware Requirement
Isaac Sim requires an NVIDIA GPU with RTX capabilities (RTX 2060 or higher recommended). If you don't have access to compatible hardware, you can:
- Follow along with the conceptual explanations and code examples
- Use cloud-based GPU instances (AWS EC2 g4dn, Google Cloud GPU VMs)
- Focus on understanding the workflow without hands-on execution
:::
\`\`\`

**Renders as**:
:::caution Hardware Requirement
Isaac Sim requires an NVIDIA GPU with RTX capabilities (RTX 2060 or higher recommended). If you don't have access to compatible hardware, you can:
- Follow along with the conceptual explanations and code examples
- Use cloud-based GPU instances (AWS EC2 g4dn, Google Cloud GPU VMs)
- Focus on understanding the workflow without hands-on execution
:::

### Info (Further Reading)

\`\`\`mdx
:::info Further Reading
For more advanced topics on Visual SLAM:
- [Cadena2016] - Comprehensive SLAM survey covering history and future directions
- [MurArtal2017] - ORB-SLAM2 original paper (CPU-based alternative to Isaac ROS)
- [IsaacROS2023] - Official Isaac ROS documentation with advanced configuration options
:::
\`\`\`

**Renders as**:
:::info Further Reading
For more advanced topics on Visual SLAM:
- [Cadena2016] - Comprehensive SLAM survey covering history and future directions
- [MurArtal2017] - ORB-SLAM2 original paper (CPU-based alternative to Isaac ROS)
- [IsaacROS2023] - Official Isaac ROS documentation with advanced configuration options
:::

## Tables

### Comparison Table Example

\`\`\`mdx
| Feature | CPU-based SLAM | Isaac ROS VSLAM |
|---------|----------------|-----------------|
| **Platform** | Any x86/ARM CPU | NVIDIA GPU (CUDA) |
| **Update Rate** | 10-30 Hz | 50-100 Hz |
| **Power** | 5-15W | 20-40W (GPU) |
| **Latency** | 50-100ms | 10-20ms |
| **Use Case** | General robotics | Real-time, high-speed robots |
\`\`\`

**Renders as**:
| Feature | CPU-based SLAM | Isaac ROS VSLAM |
|---------|----------------|-----------------|
| **Platform** | Any x86/ARM CPU | NVIDIA GPU (CUDA) |
| **Update Rate** | 10-30 Hz | 50-100 Hz |
| **Power** | 5-15W | 20-40W (GPU) |
| **Latency** | 50-100ms | 10-20ms |
| **Use Case** | General robotics | Real-time, high-speed robots |

## Images with Captions

\`\`\`mdx
<figure>
  <img
    src={require('@site/static/img/module-3/screenshots/isaac-ui-scene-hierarchy.png').default}
    alt="Isaac Sim UI showing scene hierarchy panel with USD stage tree"
  />
  <figcaption>
    <b>Figure 1.1:</b> Isaac Sim scene hierarchy panel displaying the USD stage tree structure.
    Prims (objects) are organized hierarchically with parent-child relationships.
  </figcaption>
</figure>
\`\`\`

## Tabbed Content (Alternative Configurations)

\`\`\`mdx
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
  <TabItem value="python" label="Python API" default>
    \`\`\`python
    # Python approach for launching VSLAM
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='isaac_ros_visual_slam',
                executable='visual_slam_node',
                name='visual_slam',
                parameters=[{'num_cameras': 1}]
            )
        ])
    \`\`\`
  </TabItem>
  <TabItem value="bash" label="Command Line">
    \`\`\`bash
    # Bash command for launching VSLAM
    ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \\
      num_cameras:=1 \\
      camera_frame_id:=camera_link
    \`\`\`
  </TabItem>
</Tabs>
\`\`\`

## Inline Code and Variable Highlighting

Use single backticks for inline code references:

\`\`\`mdx
The VSLAM node publishes odometry data to the \`/visual_slam/tracking/odometry\` topic.
You can configure the \`num_cameras\` parameter to support stereo setups (set to \`2\` for dual cameras).
\`\`\`

**Renders as**:
The VSLAM node publishes odometry data to the `/visual_slam/tracking/odometry` topic. You can configure the `num_cameras` parameter to support stereo setups (set to `2` for dual cameras).

## Math Equations (if needed)

For mathematical notation, use KaTeX syntax:

\`\`\`mdx
The Dynamic Window Approach minimizes a cost function:

$$
G(v, \omega) = \alpha \cdot heading(v, \omega) + \beta \cdot dist(v, \omega) + \gamma \cdot velocity(v, \omega)
$$

where $v$ is linear velocity, $\omega$ is angular velocity, and $\alpha, \beta, \gamma$ are weights.
\`\`\`

## Best Practices

### Code Block Guidelines
1. **Always include language identifier** for syntax highlighting
2. **Add comments** to explain non-obvious code sections
3. **Use realistic examples** that students can adapt
4. **Include file paths** in titles when importing from `static/code/`
5. **Validate all code** before embedding (see validation workflow in research.md:145)

### Admonition Usage
- **Tip**: Learning objectives, helpful hints
- **Note**: Prerequisites, context, clarifications
- **Warning**: Common misconceptions, pitfalls to avoid
- **Caution**: Hardware requirements, safety considerations
- **Info**: Further reading, advanced topics, external resources

### Accessibility
- Always provide **alt text** for images
- Use **semantic headings** (H2 for sections, H3 for subsections)
- Ensure **color contrast** in custom diagrams (see diagram-style-guide.md:10)
- Provide **text alternatives** for all visual content

## Validation Checklist

Before finalizing chapter content, verify:

- [ ] All code blocks have language identifiers (`python`, `yaml`, `bash`)
- [ ] External code files imported using `raw-loader` and `CodeBlock`
- [ ] All images have descriptive alt text
- [ ] Admonitions used appropriately (tip/note/warning/caution/info)
- [ ] Tables have header rows and clear column labels
- [ ] Inline code uses single backticks for topic names, parameters, file paths
- [ ] Citations formatted as `[AuthorYear]` with corresponding bibliography entry

## Example Chapter Section

\`\`\`mdx
---
sidebar_position: 1
---

# Chapter 1: Isaac Sim Synthetic Data Generation

:::tip Learning Objectives
After completing this chapter, you should be able to:
- Explain the Isaac Sim Replicator API workflow
- Identify domain randomization techniques
- Describe the value of synthetic training data
:::

## 1.1 Isaac Sim Architecture

NVIDIA Isaac Sim is a robotics simulation platform built on Omniverse, providing physically accurate rendering and sensor simulation. The architecture consists of:

\`\`\`mermaid
graph TD
    SimEngine[Simulation Engine]:::simulation --> Rendering[RTX Rendering]:::simulation
    SimEngine --> Physics[PhysX Physics]:::simulation
    Rendering --> Sensors[Sensor Simulation]:::sensor
    Physics --> Sensors
    Sensors --> ROSBridge[ROS 2 Bridge]:::data

    classDef simulation fill:#e1f5ff,stroke:#0084c7,stroke-width:2px
    classDef sensor fill:#c8e6c9,stroke:#4caf50,stroke-width:2px
    classDef data fill:#fff9c4,stroke:#fbc02d,stroke-width:2px
\`\`\`

### Key Components

1. **Simulation Engine**: USD-based scene management
2. **Rendering**: Ray-traced visuals using NVIDIA RTX
3. **Physics**: PhysX for realistic rigid body dynamics
4. **Sensor Simulation**: RGB, depth, semantic segmentation cameras

[Continue with section content...]

## References

[NVIDIA2023] NVIDIA Corporation, "Isaac Sim Documentation," 2023.
\`\`\`

## Version History

- **v1.0 (2025-12-09)**: Initial MDX component guide for Module 3
