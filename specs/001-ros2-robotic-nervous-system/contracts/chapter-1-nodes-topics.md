# Chapter 1 Contract: ROS 2 Nodes and Topic Communication

**Maps to**: User Story 1 (P1), FR-001 through FR-009
**Target Word Count**: 2500-3000 words
**File**: `Humain-robotic-book/docs/module-1-ros2/01-nodes-and-topics.mdx`

## Content Requirements

### Code Examples (4 Total)

1. **Minimal Publisher** (string messages, 1Hz timer)
   - Publishes to `/sensor_data` topic
   - Uses `std_msgs/msg/String`
   - Includes full imports, `main()` function, proper init/shutdown
   - Demonstrates: Basic node creation, publisher creation, timer callback
   - Validates: FR-002, FR-005

2. **Minimal Subscriber** (print to terminal)
   - Subscribes to `/sensor_data` topic
   - Callback prints received message using `get_logger().info()`
   - Demonstrates: Subscriber creation, callback pattern
   - Validates: FR-003, FR-005

3. **Publisher with Custom Message Content**
   - Modifies Example 1 to publish sensor readings (e.g., temperature float)
   - Shows how to change message data
   - Demonstrates: Message field modification
   - Validates: FR-009 (exercise basis)

4. **QoS Profile Demonstration** (RELIABLE vs BEST_EFFORT)
   - Two publishers: one RELIABLE, one BEST_EFFORT
   - Explanation of when to use each (sensor data vs commands)
   - Simple comparison without overwhelming detail
   - Demonstrates: QoS creation, policy differences
   - Validates: FR-006

### Diagrams (2 Required)

1. **ROS 2 Node Architecture**
   - SVG diagram showing: Node → Executor → Callbacks (timer, subscription)
   - Visual representation of event loop concept
   - Labels: `rclpy.spin()`, callback queue, execution flow

2. **Topic Publish-Subscribe Flow**
   - Publisher node → Topic (with message type label) → Subscriber node
   - Show message direction (arrows)
   - Include QoS profile annotation (RELIABLE/BEST_EFFORT)
   - Optional: Show multiple subscribers to same topic

### Exercises (3 Total)

1. **Basic: Modify Publisher Message Content**
   - Objective: Change message string from "Hello, ROS 2!" to custom text
   - Steps: Locate line with `msg.data`, change string, run and verify
   - Verification: `ros2 topic echo /sensor_data` shows new message
   - Solution: 2 lines changed (string literal)

2. **Intermediate: Change Topic Name and Connect Subscriber**
   - Objective: Rename topic from `/sensor_data` to `/robot_status`, update both nodes
   - Steps: Modify topic names in publisher and subscriber, run both, verify connection
   - Verification: `ros2 topic info /robot_status` shows 1 publisher, 1 subscriber
   - Solution: 2 lines in publisher, 1 line in subscriber

3. **Advanced: Experiment with QoS Settings**
   - Objective: Create publisher with BEST_EFFORT, subscriber with RELIABLE, observe incompatibility
   - Steps: Modify QoS profiles in Example 4, run nodes, check `ros2 topic info`
   - Verification: Subscriber does not receive messages (QoS mismatch)
   - Solution: Explanation of QoS compatibility rules, fix by matching policies

### Prerequisites List

- ROS 2 Humble Hawksbill installed and sourced (`source /opt/ros/humble/setup.bash`)
- Ubuntu 22.04 LTS (or compatible OS)
- Python 3.10 or later
- `rclpy` package (installed with ROS 2 Desktop)
- `std_msgs` package (installed with ROS 2 Desktop)
- Colcon build tool (for package creation): `sudo apt install python3-colcon-common-extensions`
- Text editor (VS Code, Vim, or nano)

### Learning Objectives (5 Measurable Outcomes)

By the end of this chapter, you will be able to:

1. **Create a ROS 2 publisher node in Python** that publishes string messages to a named topic at a specified frequency (20 minutes, validates SC-001)
2. **Create a ROS 2 subscriber node in Python** that receives and processes messages from a topic (15 minutes)
3. **Explain the difference between RELIABLE and BEST_EFFORT QoS policies** and when to use each (5 minutes)
4. **Use ROS 2 CLI tools** (`ros2 topic list`, `ros2 topic echo`, `ros2 node list`) to inspect and debug topic communication (5 minutes, validates SC-009)
5. **Modify existing code examples** to change topic names, message content, and QoS settings (10 minutes)

### Common Errors (5 Examples)

1. **Error: `Context must be initialized`**
   - Symptom: RuntimeError when creating node
   - Cause: Forgot to call `rclpy.init()` before node creation
   - Solution: Add `rclpy.init(args=args)` at start of `main()`

2. **Error: No messages received in subscriber**
   - Symptom: Subscriber runs but callback never called
   - Cause: Topic name mismatch or QoS incompatibility
   - Solution: Verify topic names match exactly; check QoS policies with `ros2 topic info`

3. **Error: `ModuleNotFoundError: No module named 'std_msgs'`**
   - Symptom: Import error when running node
   - Cause: ROS 2 environment not sourced
   - Solution: Run `source /opt/ros/humble/setup.bash` before executing node

4. **Error: Node name already in use**
   - Symptom: Warning: "Node name 'minimal_publisher' already exists..."
   - Cause: Running same node twice without unique names
   - Solution: Provide unique node names or use `--ros-args -r __node:=unique_name`

5. **Error: `KeyboardInterrupt` doesn't stop node**
   - Symptom: Ctrl+C takes long time or doesn't cleanly shutdown
   - Cause: Missing `try/except` around `rclpy.spin()` or improper shutdown
   - Solution: Add proper exception handling and ensure `node.destroy_node()` and `rclpy.shutdown()` are called

### Citations (Minimum 5, IEEE Format)

Required references:

1. [ROS2Docs2023] Open Robotics, "ROS 2 Documentation: Humble Hawksbill," docs.ros.org, https://docs.ros.org/en/humble/, 2023.
2. [rclpy2023] Open Robotics, "rclpy - ROS Client Library for Python," GitHub, https://github.com/ros2/rclpy, 2023.
3. [ROS2Concepts2023] Open Robotics, "ROS 2 Concepts: Nodes," docs.ros.org, https://docs.ros.org/en/humble/Concepts/About-Nodes.html, 2023.
4. [QoSGuide2023] Open Robotics, "About Quality of Service Settings," docs.ros.org, https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html, 2023.
5. [DDS2015] Object Management Group, "Data Distribution Service (DDS) Version 1.4," OMG Specification, https://www.omg.org/spec/DDS/, 2015.

## Acceptance Criteria

- [ ] Chapter MDX file exists at `docs/module-1-ros2/01-nodes-and-topics.mdx`
- [ ] Word count: 2500-3000 words (±10% acceptable)
- [ ] All 4 code examples present and tested in `ros2_code_examples/`
- [ ] Both diagrams created and saved in `docs/module-1-ros2/assets/`
- [ ] All 3 exercises with solutions (Basic, Intermediate, Advanced)
- [ ] Common Errors section with 5 examples
- [ ] References section with ≥5 IEEE citations
- [ ] Docusaurus build succeeds: `npm run build` exits 0
- [ ] All code examples run on ROS 2 Humble (Ubuntu 22.04, Python 3.10)
- [ ] PEP 8 compliance: `black --check` and `flake8` pass
- [ ] Validates FR-001 through FR-009
- [ ] Supports SC-001 (student creates pub-sub pair in 20 min)
