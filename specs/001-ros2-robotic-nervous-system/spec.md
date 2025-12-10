# Feature Specification: Module 1 – ROS 2 Robotic Nervous System

**Feature Branch**: `001-ros2-robotic-nervous-system`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Module 1 – ROS 2 Robotic Nervous System - Target audience: Beginner/intermediate robotics students. Focus: ROS 2 nodes, topics, services, rclpy control, and humanoid URDF basics. Success criteria: 2–3 chapters explaining ROS 2 fundamentals, Students can run nodes, use topics/services, and connect Python agents via rclpy, Students can read and edit a simple humanoid URDF. Constraints: Markdown format, Include runnable ROS 2 code, Keep explanations simple and accurate. Not building: Nav2, perception, or full simulation systems."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understanding ROS 2 Node Communication (Priority: P1)

A robotics student needs to understand how ROS 2 nodes communicate using topics and services to build distributed robot control systems. They start by learning the publish-subscribe pattern through concrete examples like sensor data streaming and command messaging.

**Why this priority**: Core foundation for all ROS 2 development. Without understanding node communication, students cannot progress to any other robotics concepts. This is the absolute minimum viable content.

**Independent Test**: Student can create two nodes - one publisher sending sensor data (e.g., temperature readings) and one subscriber processing that data - then verify communication using `ros2 topic echo` and `ros2 node list` commands. Delivers immediate understanding of distributed systems.

**Acceptance Scenarios**:

1. **Given** student has ROS 2 Humble installed, **When** they run the provided publisher example code, **Then** they see messages published to `/sensor_data` topic using `ros2 topic echo /sensor_data`
2. **Given** publisher node is running, **When** student runs the subscriber example, **Then** subscriber prints received messages to terminal with correct data values
3. **Given** both nodes are running, **When** student uses `ros2 node list`, **Then** both node names appear in the list
4. **Given** student wants to inspect message flow, **When** they run `ros2 topic info /sensor_data`, **Then** they see publisher and subscriber counts matching their running nodes
5. **Given** student modifies the message content in publisher, **When** they restart the publisher, **Then** subscriber receives and displays the modified content

---

### User Story 2 - Implementing Request-Response Patterns with Services (Priority: P2)

A student needs to implement synchronous request-response communication for tasks like configuration changes, calibration requests, or control mode switching. They learn service client-server patterns through practical examples.

**Why this priority**: Services are essential for robot control commands (e.g., "move to position", "change mode"). Builds on topic knowledge but adds synchronous communication patterns. Can be tested independently of URDF content.

**Independent Test**: Student can create a service server that performs a calculation (e.g., adding two numbers or converting units) and a client that sends requests, then verify responses are correct. Demonstrates understanding without requiring hardware or complex simulation.

**Acceptance Scenarios**:

1. **Given** student has completed topic examples, **When** they run the service server example, **Then** server starts and waits for requests as shown by `ros2 service list`
2. **Given** service server is running, **When** student runs the client example with test parameters, **Then** client receives correct response and prints result
3. **Given** student wants to test manually, **When** they use `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"`, **Then** they receive response `{sum: 12}`
4. **Given** server is under load, **When** student runs multiple client requests sequentially, **Then** each request receives the correct response in order
5. **Given** student modifies service logic, **When** they restart server and send test request, **Then** response reflects the modified logic

---

### User Story 3 - Understanding Humanoid Robot Structure with URDF (Priority: P3)

A student needs to understand robot physical structure representation using URDF (Unified Robot Description Format) to read, interpret, and make basic modifications to humanoid robot models.

**Why this priority**: URDF knowledge is crucial for robot modeling but can be learned independently after communication concepts. Enables students to understand existing robot descriptions and make simple modifications without requiring full simulation setup.

**Independent Test**: Student can read a provided simple humanoid URDF file (e.g., 2-link arm or torso with head), identify key components (links, joints, frames), modify joint limits or link dimensions, and validate the URDF using `check_urdf` command. Delivers understanding without requiring Gazebo or RViz setup.

**Acceptance Scenarios**:

1. **Given** student has a simple humanoid URDF file (head, torso, two arms), **When** they open it in a text editor, **Then** they can identify all link elements, joint elements, and their relationships
2. **Given** student reads URDF documentation in the chapter, **When** they locate a revolute joint in the URDF, **Then** they can identify joint limits (upper, lower), axis of rotation, and parent-child relationships
3. **Given** student wants to validate URDF, **When** they run `check_urdf simple_humanoid.urdf`, **Then** command outputs "robot name is: [name]" and shows the link tree structure
4. **Given** student wants to modify joint range, **When** they change upper limit value for shoulder joint and save file, **Then** `check_urdf` still reports valid URDF
5. **Given** student wants to visualize structure, **When** they run `urdf_to_graphiz simple_humanoid.urdf`, **Then** tool generates a PDF showing link-joint tree diagram

---

### Edge Cases

- **What happens when** a subscriber node starts before the publisher? (Answer: Subscriber waits until publisher comes online, then begins receiving messages - students verify with delayed publisher start)
- **How does system handle** service call timeout when server is unresponsive? (Answer: Client times out after configurable duration - students test by not starting server or killing it mid-request)
- **What happens when** URDF has invalid syntax (missing closing tag, incorrect joint type)? (Answer: `check_urdf` reports specific error with line information - students debug common errors)
- **How does system handle** circular parent-child relationships in URDF? (Answer: `check_urdf` detects and reports loops - students understand tree structure requirement)
- **What happens when** message types don't match between publisher and subscriber? (Answer: ROS 2 type checking prevents communication - students learn about interface definitions)

## Requirements *(mandatory)*

### Functional Requirements

#### Chapter 1: ROS 2 Nodes and Topic Communication
- **FR-001**: Chapter MUST explain ROS 2 node concept with concrete definition and real-world robotics analogy (e.g., "nodes are independent processes like camera driver, motor controller")
- **FR-002**: Chapter MUST provide complete, runnable Python publisher code using rclpy that publishes string messages to a named topic at 1Hz
- **FR-003**: Chapter MUST provide complete, runnable Python subscriber code using rclpy that subscribes to the same topic and prints received messages
- **FR-004**: Chapter MUST include package structure showing `package.xml`, `setup.py` dependencies, and proper package organization for ROS 2 Python packages
- **FR-005**: Code examples MUST include all necessary imports (`import rclpy`, `from rclpy.node import Node`, etc.) and not omit setup code
- **FR-006**: Chapter MUST explain QoS (Quality of Service) basics with simple default example (RELIABLE vs BEST_EFFORT) without overwhelming beginners
- **FR-007**: Chapter MUST include step-by-step terminal commands for running nodes: `ros2 run <package> <node>`
- **FR-008**: Chapter MUST provide debugging commands: `ros2 topic list`, `ros2 topic echo`, `ros2 topic info`, `ros2 node list`
- **FR-009**: Chapter MUST include at least one practical exercise where student modifies message content or topic name and verifies changes

#### Chapter 2: Services for Request-Response Communication
- **FR-010**: Chapter MUST explain service concept and contrast with topics (synchronous vs asynchronous, request-response vs streaming)
- **FR-011**: Chapter MUST provide complete, runnable Python service server code using rclpy with defined service interface (e.g., `AddTwoInts` from `example_interfaces`)
- **FR-012**: Chapter MUST provide complete, runnable Python service client code using rclpy that sends request and handles response
- **FR-013**: Service examples MUST demonstrate error handling for service availability check (`client.wait_for_service()` with timeout)
- **FR-014**: Chapter MUST explain service interface definition structure (`.srv` files) with example showing request and response fields
- **FR-015**: Chapter MUST include commands for service introspection: `ros2 service list`, `ros2 service type`, `ros2 service call`
- **FR-016**: Chapter MUST include practical exercise where student creates custom service logic (e.g., unit conversion, simple calculation)

#### Chapter 3: Humanoid URDF Basics
- **FR-017**: Chapter MUST explain URDF purpose (robot geometric and kinematic description) with humanoid robotics context
- **FR-018**: Chapter MUST provide annotated simple humanoid URDF example with minimum 4 links (base/torso, head, left arm, right arm) and 3 joints
- **FR-019**: URDF example MUST include XML comments explaining each section: `<robot>`, `<link>`, `<joint>`, `<visual>`, `<collision>`, `<inertial>`
- **FR-020**: Chapter MUST explain joint types relevant to humanoids: `revolute` (shoulder, elbow), `fixed` (torso to base), `continuous` (head pan)
- **FR-021**: Chapter MUST explain link coordinate frames and parent-child relationships using tree diagram
- **FR-022**: Chapter MUST include `check_urdf` validation workflow with example valid and invalid URDF to demonstrate debugging
- **FR-023**: Chapter MUST explain joint limits (upper, lower, effort, velocity) with concrete humanoid examples (shoulder joint: -π/2 to π/2 radians)
- **FR-024**: Chapter MUST include practical exercise: student modifies joint limit, adds a new link (e.g., simple hand), or changes link dimensions
- **FR-025**: Chapter MUST provide visualization command using `urdf_to_graphiz` to generate link tree diagram

#### Cross-Chapter Requirements
- **FR-026**: All code examples MUST be tested on ROS 2 Humble (Ubuntu 22.04) and include Python 3.10+ syntax
- **FR-027**: Each chapter MUST include "Prerequisites" section listing required ROS 2 packages and system dependencies
- **FR-028**: Each chapter MUST include "Learning Objectives" section with 3-5 measurable outcomes
- **FR-029**: All code MUST follow PEP 8 Python style guidelines and include docstrings for classes and key methods
- **FR-030**: Chapters MUST include "Common Errors" section showing typical mistakes (e.g., forgetting `rclpy.init()`, incorrect package.xml) and solutions
- **FR-031**: Each chapter MUST cite official ROS 2 documentation using IEEE format: `[ROS2Docs2023] "ROS 2 Documentation: Humble," docs.ros.org, 2023`
- **FR-032**: Markdown content MUST include code blocks with syntax highlighting: ` ```python ` for code, ` ```bash ` for terminal commands, ` ```xml ` for URDF

### Key Entities

- **ROS 2 Node**: Independent executable process that performs computation. Has unique name, can publish/subscribe to topics, provide/call services. Attributes: node name, namespace, parameter list, topic/service endpoints.
- **Topic**: Named communication channel for asynchronous message streaming. Attributes: topic name, message type, QoS profile, publisher count, subscriber count.
- **Message**: Data structure transmitted over topics. Attributes: message type (e.g., `std_msgs/msg/String`), fields with typed data, timestamp (for stamped messages).
- **Service**: Synchronous request-response communication mechanism. Attributes: service name, service type (`.srv` definition), request fields, response fields.
- **URDF Link**: Rigid body component of robot. Attributes: link name, visual geometry, collision geometry, inertial properties (mass, inertia tensor), frame origin.
- **URDF Joint**: Connection between two links defining kinematic relationship. Attributes: joint name, joint type (revolute/continuous/prismatic/fixed), parent link, child link, axis of motion, joint limits (position, velocity, effort).
- **ROS 2 Package**: Organizational unit for code and resources. Attributes: package name, dependencies (`package.xml`), build configuration (`setup.py`), source files, launch files.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Student can create and run a ROS 2 publisher-subscriber pair within 20 minutes after reading Chapter 1, verified by successful `ros2 topic echo` output
- **SC-002**: Student can successfully call a ROS 2 service from command line and from Python client within 15 minutes after reading Chapter 2, verified by correct response output
- **SC-003**: Student can read a 10-link humanoid URDF and identify all joint types, parent-child relationships, and joint limits within 10 minutes after reading Chapter 3
- **SC-004**: Student can modify a URDF joint limit and validate the change using `check_urdf` without errors within 5 minutes after completing Chapter 3 exercises
- **SC-005**: 90% of students successfully complete all chapter exercises on first attempt (measured by exercise submission success rate or self-assessment checklist)
- **SC-006**: All code examples run without errors on fresh ROS 2 Humble installation (Ubuntu 22.04, Python 3.10) verified by CI test pipeline
- **SC-007**: Each chapter contains minimum 5 IEEE-format citations to official ROS 2 documentation or robotics research papers
- **SC-008**: Student can explain the difference between topics and services in their own words after completing Chapters 1-2 (measured by comprehension quiz: 80% pass rate)
- **SC-009**: Student can trace message flow from publisher to subscriber using ROS 2 introspection tools (`ros2 topic`, `ros2 node`) within 5 minutes
- **SC-010**: Module content completion time averages 4-6 hours for beginner student (measured by pilot reader feedback and self-reported time)
