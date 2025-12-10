# ROS 2 Integration Patterns for VLA Pipeline

**Feature**: `004-vla-pipeline`
**Date**: 2025-12-09
**Purpose**: Document ROS 2 integration architecture, topic flow, action servers, and node communication patterns for Vision-Language-Action systems

---

## Overview

This document describes how VLA (Vision-Language-Action) components integrate with ROS 2 Humble to create an end-to-end voice-controlled humanoid robot system. The architecture follows ROS 2 best practices for distributed node communication, action-based task execution, and sensor feedback loops.

---

## System Architecture

### High-Level Component Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            VLA Pipeline Architecture                        │
└─────────────────────────────────────────────────────────────────────────────┘

┌──────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Microphone  │────▶│ audio_       │────▶│   whisper_   │────▶│ llm_planner_ │
│    (HW)      │     │ capture_node │     │     node     │     │     node     │
└──────────────┘     └──────────────┘     └──────────────┘     └──────────────┘
                            │                     │                      │
                            ▼                     ▼                      ▼
                     /audio/input      /voice_command/     /cognitive_plan/
                                       transcription           output
                                                                   │
                                                                   ▼
                                                          ┌──────────────┐
                                                          │   action_    │
                                                          │ orchestrator │
                                                          │     node     │
                                                          └──────────────┘
                                                                   │
         ┌─────────────────────────────────────────────────────────┼───────────┐
         │                                                         │           │
         ▼                                                         ▼           ▼
┌──────────────┐                                          ┌──────────────┐   ┌───────────┐
│   Nav2       │                                          │   vision_    │   │ MoveIt2   │
│  Controller  │                                          │  detector    │   │ Grasping  │
└──────────────┘                                          └──────────────┘   └───────────┘
         │                                                         │              │
         ▼                                                         ▼              ▼
┌──────────────┐                                          ┌──────────────┐   ┌───────────┐
│  Robot Base  │                                          │   Camera     │   │  Gripper  │
│  (Hardware)  │                                          │  (Hardware)  │   │(Hardware) │
└──────────────┘                                          └──────────────┘   └───────────┘
```

---

## Node Descriptions

### 1. audio_capture_node

**Package**: `audio_common` (ROS 2)
**Purpose**: Capture audio from microphone and publish as ROS 2 messages

**Published Topics**:
- `/audio/input` (`audio_msgs/Audio`): Raw audio stream from microphone
  - QoS: BEST_EFFORT, VOLATILE (real-time stream)
  - Publish Rate: Continuous (typically 16kHz sample rate, chunked into 100ms segments)

**Parameters**:
- `device` (string): Audio device ID (default: `"default"`)
- `sample_rate` (int): Sample rate in Hz (default: `16000`)
- `channels` (int): Number of audio channels (default: `1` for mono)
- `format` (string): Audio format (default: `"S16LE"` - 16-bit PCM Little Endian)
- `chunk_size` (int): Audio chunk size in samples (default: `1600` - 100ms at 16kHz)

**Example Launch**:
```bash
ros2 run audio_common audio_capture_node --ros-args \
  -p device:="hw:0,0" \
  -p sample_rate:=16000 \
  -p channels:=1
```

---

### 2. whisper_node

**Package**: `whisper_ros2` (custom implementation)
**Purpose**: Transcribe audio to text using OpenAI Whisper model

**Subscribed Topics**:
- `/audio/input` (`audio_msgs/Audio`): Audio stream from audio_capture_node
  - QoS: BEST_EFFORT, VOLATILE

**Published Topics**:
- `/voice_command/transcription` (`std_msgs/String`): Transcribed text
  - QoS: RELIABLE, TRANSIENT_LOCAL (ensure transcriptions not lost)
  - Publish Rate: On detection of speech (variable, typically every 2-5 seconds)

**Parameters**:
- `model_name` (string): Whisper model variant (default: `"base"`)
  - Options: `"tiny"`, `"base"`, `"small"`, `"medium"`, `"large"`
- `language` (string): Target language code (default: `"en"`)
- `confidence_threshold` (float): Minimum confidence to publish (default: `0.7`)
- `device` (string): Compute device (default: `"cuda"` if available, else `"cpu"`)
- `buffer_duration_s` (float): Audio buffer duration in seconds (default: `3.0`)

**Conceptual Implementation**:
```python
import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
from std_msgs.msg import String
import whisper
import numpy as np

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Parameters
        self.declare_parameter('model_name', 'base')
        self.declare_parameter('confidence_threshold', 0.7)
        model_name = self.get_parameter('model_name').value

        # Load Whisper model
        self.model = whisper.load_model(model_name)
        self.get_logger().info(f'Loaded Whisper model: {model_name}')

        # ROS 2 interfaces
        self.subscription = self.create_subscription(
            Audio,
            '/audio/input',
            self.audio_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            '/voice_command/transcription',
            10
        )

        # Audio buffer
        self.audio_buffer = []

    def audio_callback(self, msg):
        # Append audio data to buffer
        audio_array = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_buffer.extend(audio_array)

        # Process when buffer reaches threshold (e.g., 3 seconds)
        if len(self.audio_buffer) >= msg.info.sample_rate * 3:
            self.process_audio()

    def process_audio(self):
        # Convert buffer to float32 for Whisper
        audio_float = np.array(self.audio_buffer, dtype=np.float32) / 32768.0

        # Run Whisper inference
        result = self.model.transcribe(audio_float)
        transcription = result['text']
        confidence = result.get('confidence', 1.0)  # Not all Whisper versions return confidence

        # Publish if above confidence threshold
        if confidence >= self.get_parameter('confidence_threshold').value:
            msg = String()
            msg.data = transcription
            self.publisher.publish(msg)
            self.get_logger().info(f'Transcription: {transcription} (confidence: {confidence:.2f})')

        # Clear buffer
        self.audio_buffer = []
```

---

### 3. llm_planner_node

**Package**: `llm_ros2_planner` (custom implementation)
**Purpose**: Decompose natural language commands into structured robot action plans using LLM

**Subscribed Topics**:
- `/voice_command/transcription` (`std_msgs/String`): Transcribed commands from Whisper
  - QoS: RELIABLE, TRANSIENT_LOCAL

**Published Topics**:
- `/cognitive_plan/output` (`std_msgs/String` - JSON serialized): Structured action plan
  - QoS: RELIABLE, TRANSIENT_LOCAL
  - Publish Rate: On receiving transcription (1-5 seconds latency)

**Parameters**:
- `llm_provider` (string): LLM API provider (default: `"openai"`)
  - Options: `"openai"`, `"anthropic"`, `"local_llama"`
- `model_name` (string): Model identifier (default: `"gpt-4"`)
- `temperature` (float): LLM temperature for determinism (default: `0.2`)
- `system_prompt_path` (string): Path to system prompt file (default: `"~/system_prompt.txt"`)
- `timeout_s` (float): LLM API timeout in seconds (default: `10.0`)

**Conceptual Implementation**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import openai  # Or anthropic, llama.cpp

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Parameters
        self.declare_parameter('model_name', 'gpt-4')
        self.declare_parameter('temperature', 0.2)
        self.declare_parameter('system_prompt_path', '~/system_prompt.txt')

        # Load system prompt
        prompt_path = self.get_parameter('system_prompt_path').value
        with open(prompt_path, 'r') as f:
            self.system_prompt = f.read()

        # ROS 2 interfaces
        self.subscription = self.create_subscription(
            String,
            '/voice_command/transcription',
            self.command_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            '/cognitive_plan/output',
            10
        )

    def command_callback(self, msg):
        user_command = msg.data
        self.get_logger().info(f'Received command: {user_command}')

        # Call LLM API
        try:
            plan = self.generate_plan(user_command)

            # Validate plan
            if self.validate_plan(plan):
                # Publish plan as JSON
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.publisher.publish(plan_msg)
                self.get_logger().info(f'Published plan with {len(plan["steps"])} steps')
            else:
                self.get_logger().error('Plan validation failed')
        except Exception as e:
            self.get_logger().error(f'LLM planning error: {str(e)}')

    def generate_plan(self, command):
        # OpenAI API example
        response = openai.ChatCompletion.create(
            model=self.get_parameter('model_name').value,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=self.get_parameter('temperature').value
        )
        plan_json = json.loads(response.choices[0].message.content)
        return plan_json

    def validate_plan(self, plan):
        # Basic validation
        if 'steps' not in plan:
            return False
        for step in plan['steps']:
            if 'action' not in step or 'params' not in step:
                return False
        return True
```

---

### 4. action_orchestrator_node

**Package**: `vla_orchestrator` (custom implementation)
**Purpose**: Convert cognitive plans into ROS 2 action calls and manage execution

**Subscribed Topics**:
- `/cognitive_plan/output` (`std_msgs/String` - JSON): Plans from LLM
  - QoS: RELIABLE, TRANSIENT_LOCAL

**Published Topics**:
- `/vla/feedback` (`vla_msgs/Feedback`): Execution status and sensor feedback
  - QoS: RELIABLE, VOLATILE
  - Publish Rate: On action state changes (variable)

**Action Clients**:
- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`): Navigation actions
- `/detect_objects` (`vision_msgs/action/DetectObjects`): Object detection
- `/grasp_object` (`manipulation_msgs/action/GraspObject`): Manipulation actions
- `/place_object` (`manipulation_msgs/action/PlaceObject`): Object placement

**Parameters**:
- `max_concurrent_actions` (int): Max parallel actions (default: `1` - sequential)
- `retry_on_failure` (bool): Retry failed actions (default: `false`)
- `action_timeout_multiplier` (float): Multiply expected_duration_s by this for timeout (default: `2.0`)

**Conceptual Implementation**:
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
import json

class ActionOrchestratorNode(Node):
    def __init__(self):
        super().__init__('action_orchestrator_node')

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.detect_client = ActionClient(self, DetectObjects, '/detect_objects')
        self.grasp_client = ActionClient(self, GraspObject, '/grasp_object')

        # Subscribe to plans
        self.subscription = self.create_subscription(
            String,
            '/cognitive_plan/output',
            self.plan_callback,
            10
        )

        self.current_plan = None
        self.current_step_idx = 0

    def plan_callback(self, msg):
        plan = json.loads(msg.data)
        self.get_logger().info(f'Received plan: {plan["task"]}')
        self.current_plan = plan
        self.current_step_idx = 0
        self.execute_next_step()

    def execute_next_step(self):
        if self.current_step_idx >= len(self.current_plan['steps']):
            self.get_logger().info('Plan complete!')
            return

        step = self.current_plan['steps'][self.current_step_idx]
        action_name = step['action']
        params = step['params']

        self.get_logger().info(f'Executing step {step["step_id"]}: {action_name}')

        # Route to appropriate action client
        if action_name == 'navigate_to_pose':
            self.execute_navigation(params)
        elif action_name == 'detect_objects':
            self.execute_detection(params)
        elif action_name == 'grasp_object':
            self.execute_grasp(params)
        # ... other actions

    def execute_navigation(self, params):
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = params['x']
        goal.pose.pose.position.y = params['y']
        # ... set orientation from theta

        self.nav_client.send_goal_async(goal).add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            self.current_step_idx += 1
            self.execute_next_step()
        else:
            self.get_logger().error('Navigation failed')
```

---

## Topic Communication Patterns

### Pattern 1: Real-Time Streaming (Audio Input)

```
Audio Capture Node                 Whisper Node
      │                                 │
      │  /audio/input (BEST_EFFORT)     │
      ├─────────────────────────────────▶│
      │  [audio chunks @ 10Hz]          │
      │                                 │
      ├─────────────────────────────────▶│
      │                                 │
      ├─────────────────────────────────▶│
```

**QoS Profile**:
- **Reliability**: BEST_EFFORT (dropped messages acceptable for real-time)
- **Durability**: VOLATILE (only current data matters)
- **History Depth**: 10 (buffer recent messages)

### Pattern 2: Command Messaging (Transcriptions, Plans)

```
Whisper Node                      LLM Planner Node
      │                                 │
      │ /voice_command/transcription    │
      │         (RELIABLE)              │
      ├─────────────────────────────────▶│
      │  "Bring me a drink"             │
      │                                 │
      │                                 │ [LLM processing 2-5s]
      │                                 │
      │                                 ▼
LLM Planner Node              Action Orchestrator Node
      │                                 │
      │ /cognitive_plan/output          │
      │         (RELIABLE)              │
      ├─────────────────────────────────▶│
      │  {"steps": [...]}               │
```

**QoS Profile**:
- **Reliability**: RELIABLE (messages must not be lost)
- **Durability**: TRANSIENT_LOCAL (late subscribers get last message)
- **History Depth**: 5 (retain recent commands)

### Pattern 3: Action Execution (Orchestrator → Action Servers)

```
Action Orchestrator              Nav2 Action Server
      │                                 │
      │  SendGoal(NavigateToPose)       │
      ├─────────────────────────────────▶│
      │                                 │
      │◀─────────────────────────────────┤
      │  GoalAccepted                   │
      │                                 │ [Execution starts]
      │◀─────────────────────────────────┤
      │  Feedback (distance_remaining)  │
      │                                 │
      │◀─────────────────────────────────┤
      │  Feedback (distance_remaining)  │
      │                                 │
      │◀─────────────────────────────────┤
      │  Result (SUCCESS)               │
```

**Action Interface**: ROS 2 Actions (goal-oriented, feedback-enabled, preemptable)

---

## Integration with Nav2

### Nav2 NavigateToPose Action

**Action Name**: `/navigate_to_pose`
**Message Type**: `nav2_msgs/action/NavigateToPose`

**Goal**:
```yaml
geometry_msgs/PoseStamped pose
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id: "map"
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z: 0.0
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
```

**Result**:
```yaml
std_msgs/Empty result
```

**Feedback**:
```yaml
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
```

**Example Usage in Action Orchestrator**:
```python
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

def navigate_to_kitchen(self):
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = self.get_clock().now().to_msg()

    # Set position
    goal.pose.pose.position.x = 5.0
    goal.pose.pose.position.y = 3.0
    goal.pose.pose.position.z = 0.0

    # Set orientation (theta=0.0 → facing +X)
    theta = 0.0
    goal.pose.pose.orientation.z = math.sin(theta / 2.0)
    goal.pose.pose.orientation.w = math.cos(theta / 2.0)

    # Send goal
    self.nav_client.wait_for_server()
    future = self.nav_client.send_goal_async(goal, feedback_callback=self.nav_feedback_callback)
    future.add_done_callback(self.nav_goal_response_callback)
```

---

## Integration with Vision (Isaac ROS / YOLO)

### Custom DetectObjects Action

**Action Name**: `/detect_objects`
**Message Type**: `vision_msgs/action/DetectObjects` (custom)

**Goal**:
```yaml
string[] classes              # ["cup", "bottle", "glass"]
float32 min_confidence        # 0.8
int32 timeout_s               # 5
```

**Result**:
```yaml
vision_msgs/DetectedObject[] objects
  string id
  string class_name
  float32 confidence
  vision_msgs/BoundingBox2D bounding_box
  geometry_msgs/Point position_3d
```

**Feedback**:
```yaml
int32 detections_count
int32 processing_time_ms
```

**Example Usage**:
```python
def detect_cups(self):
    goal = DetectObjects.Goal()
    goal.classes = ['cup', 'bottle', 'glass']
    goal.min_confidence = 0.8
    goal.timeout_s = 5

    future = self.detect_client.send_goal_async(goal)
    future.add_done_callback(self.detect_result_callback)

def detect_result_callback(self, future):
    result = future.result().result
    if len(result.objects) > 0:
        self.get_logger().info(f'Detected {len(result.objects)} objects')
        for obj in result.objects:
            self.get_logger().info(f'  - {obj.class_name} at ({obj.position_3d.x}, {obj.position_3d.y}) confidence={obj.confidence}')
```

---

## Integration with MoveIt2 (Manipulation)

### Custom GraspObject Action

**Action Name**: `/grasp_object`
**Message Type**: `manipulation_msgs/action/GraspObject` (custom)

**Goal**:
```yaml
string object_id              # "obj_0"
geometry_msgs/Pose grasp_pose # Optional pre-computed grasp
float32 grasp_force_n         # 5.0
```

**Result**:
```yaml
bool success
string error_message
float32 grasp_quality         # 0.0-1.0
```

**Feedback**:
```yaml
string current_stage          # "approaching", "grasping", "lifting"
float32 grasp_quality
```

---

## Data Flow Summary

### Complete VLA Pipeline Message Flow

```
1. Microphone → audio_capture_node
   Topic: /audio/input (audio_msgs/Audio)
   Rate: 10 Hz (100ms chunks)

2. audio_capture_node → whisper_node
   Topic: /audio/input (audio_msgs/Audio)
   Rate: Continuous

3. whisper_node → llm_planner_node
   Topic: /voice_command/transcription (std_msgs/String)
   Rate: On speech detection (~every 3-5s)

4. llm_planner_node → action_orchestrator_node
   Topic: /cognitive_plan/output (std_msgs/String - JSON)
   Rate: On command received (~2-5s latency)

5. action_orchestrator_node → Nav2/Vision/MoveIt2
   Actions: NavigateToPose, DetectObjects, GraspObject
   Rate: Sequential (one action at a time)

6. Nav2/Vision/MoveIt2 → Feedback → llm_planner_node (optional)
   Topic: /vla/feedback (vla_msgs/Feedback)
   Rate: On state changes (for replanning)
```

---

## Launch File Example

**File**: `vla_pipeline.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Audio capture
        Node(
            package='audio_common',
            executable='audio_capture_node',
            name='audio_capture',
            parameters=[{
                'device': 'default',
                'sample_rate': 16000,
                'channels': 1
            }]
        ),

        # Whisper transcription
        Node(
            package='whisper_ros2',
            executable='whisper_node',
            name='whisper',
            parameters=[{
                'model_name': 'base',
                'confidence_threshold': 0.7,
                'device': 'cuda'
            }]
        ),

        # LLM planner
        Node(
            package='llm_ros2_planner',
            executable='llm_planner_node',
            name='llm_planner',
            parameters=[{
                'model_name': 'gpt-4',
                'temperature': 0.2,
                'system_prompt_path': '/path/to/system_prompt.txt'
            }]
        ),

        # Action orchestrator
        Node(
            package='vla_orchestrator',
            executable='action_orchestrator_node',
            name='action_orchestrator',
            parameters=[{
                'max_concurrent_actions': 1,
                'retry_on_failure': False
            }]
        ),
    ])
```

**Run**:
```bash
ros2 launch vla_pipeline vla_pipeline.launch.py
```

---

## Quality of Service (QoS) Recommendations

### Real-Time Streams (Audio, Video)
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
```

### Commands and Plans (Transcriptions, Cognitive Plans)
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=5
)
```

### Status Updates (Feedback, System Status)
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1
)
```

---

## Error Handling and Recovery

### Transcription Failures
- **Scenario**: Whisper fails to transcribe (low confidence <0.7)
- **Recovery**: Log warning, wait for next audio chunk, do not publish garbage transcription

### LLM Timeout
- **Scenario**: LLM API call takes >10 seconds or fails
- **Recovery**: Retry once, then publish error feedback to user ("Planning timed out, please repeat command")

### Action Failure
- **Scenario**: Navigation fails (path blocked), grasp fails (object slipped)
- **Recovery**: Publish failure feedback, optionally send back to LLM for replanning

---

## Testing and Validation

### Unit Testing Nodes

**Test Whisper Node**:
```bash
# Play test audio file
ros2 bag play test_audio.bag

# Check transcription output
ros2 topic echo /voice_command/transcription
```

**Test LLM Planner Node**:
```bash
# Publish test command
ros2 topic pub /voice_command/transcription std_msgs/String "data: 'Bring me a cup'"

# Check plan output
ros2 topic echo /cognitive_plan/output
```

### Integration Testing

**Full Pipeline Test**:
```bash
# Launch all nodes
ros2 launch vla_pipeline vla_pipeline.launch.py

# Speak command into microphone or play audio file
ros2 bag play test_commands.bag

# Monitor execution
ros2 topic echo /vla/feedback
```

---

## Summary

**ROS 2 Integration Patterns for VLA**:
1. **Audio Capture**: Microphone → audio_capture_node → /audio/input (BEST_EFFORT stream)
2. **Speech Recognition**: whisper_node subscribes to audio, publishes transcriptions (RELIABLE)
3. **Cognitive Planning**: llm_planner_node receives text, outputs structured JSON plans (RELIABLE)
4. **Action Execution**: action_orchestrator_node sends goals to Nav2, Vision, MoveIt2 (ROS 2 Actions)
5. **Feedback Loop**: Action results published to /vla/feedback for monitoring and replanning

**Key Technologies**:
- ROS 2 Topics for asynchronous messaging
- ROS 2 Actions for goal-oriented execution with feedback
- QoS Profiles for reliability/latency tradeoffs
- Nav2 for navigation, Isaac ROS/YOLO for vision, MoveIt2 for manipulation

---

**Integration Pattern Complete**: Ready for use in Module 4 educational content and code examples
