# Chapter 2 Contract: Services for Request-Response Communication

**Maps to**: User Story 2 (P2), FR-010 through FR-016
**Target Word Count**: 2000-2500 words
**File**: `Humain-robotic-book/docs/module-1-ros2/02-services.mdx`

## Content Requirements

### Code Examples (4 Total)

1. **Service Server** (AddTwoInts from example_interfaces)
   - Implements addition service: takes two integers, returns sum
   - Uses `example_interfaces/srv/AddTwoInts`
   - Includes service callback with logging
   - Demonstrates: Service creation, request/response handling
   - Validates: FR-011

2. **Service Client** (synchronous call)
   - Calls `add_two_ints` service with hardcoded values
   - Uses `call_async()` with `spin_until_future_complete()`
   - Prints result to terminal
   - Demonstrates: Client creation, async call pattern, result handling
   - Validates: FR-012

3. **Service with Timeout Handling**
   - Client waits for service with timeout (`wait_for_service(timeout_sec=5.0)`)
   - Logs "Service not available" if timeout
   - Demonstrates: Error handling, service discovery
   - Validates: FR-013

4. **Custom Service Logic** (unit conversion example)
   - Server converts Celsius to Fahrenheit (or meters to feet)
   - Uses same AddTwoInts interface creatively (input=celsius*100, output=fahrenheit*100)
   - OR: Describe how to create custom `.srv` file (without full implementation)
   - Demonstrates: Modifying service logic, alternative use cases
   - Validates: FR-016 (exercise basis)

### Diagrams (1 Required)

1. **Service Request-Response Sequence**
   - Sequence diagram or flowchart: Client → Request → Server → Compute → Response → Client
   - Show synchronous nature (client blocks waiting for response)
   - Include timeout scenario (dotted line for timeout path)
   - **Options**: Mermaid sequence diagram (simpler) or SVG (more detailed)

### Exercises (3 Total)

1. **Basic: Modify Service Computation Logic**
   - Objective: Change AddTwoInts server to multiply instead of add
   - Steps: Locate callback, change `+` to `*`, run server and client, verify result
   - Verification: Client request `(a=3, b=4)` returns `sum=12` (product)
   - Solution: 1 line changed in server callback

2. **Intermediate: Create Client with Timeout Handling**
   - Objective: Modify client to wait 10 seconds for service, then exit gracefully if unavailable
   - Steps: Add `wait_for_service()` loop, add timeout counter, log "Service unavailable" and exit
   - Verification: Run client without server, observe timeout message after 10s
   - Solution: 5-7 lines added (while loop, timeout logic, early exit)

3. **Advanced: Implement Custom Service (String Reversal)**
   - Objective: Create service that reverses a string (describe .srv file structure)
   - Steps: (Conceptual, no full implementation) Define `.srv` file, implement server callback, test with client
   - Verification: Service call with `"hello"` returns `"olleh"`
   - Solution: Outline .srv file structure, pseudo-code for server (or full code if time permits)

### Prerequisites List

- Completion of Chapter 1 (ROS 2 Nodes and Topics)
- ROS 2 Humble with `example_interfaces` package (installed with Desktop)
- Understanding of synchronous vs asynchronous communication
- Familiarity with Python functions and return values

### Learning Objectives (4 Measurable Outcomes)

By the end of this chapter, you will be able to:

1. **Explain the difference between services and topics** (synchronous request-response vs asynchronous streaming) in your own words (5 minutes, validates SC-008)
2. **Create a ROS 2 service server in Python** that responds to client requests (15 minutes)
3. **Create a ROS 2 service client in Python** that sends requests and handles responses, including timeouts (15 minutes, validates SC-002)
4. **Use ROS 2 CLI tools** (`ros2 service list`, `ros2 service call`) to inspect and test services manually (5 minutes)

### Common Errors (4 Examples)

1. **Error: Service call hangs indefinitely**
   - Symptom: Client blocks forever, no response
   - Cause: Service server not running or wrong service name
   - Solution: Check server is running (`ros2 service list`), verify service name matches exactly

2. **Error: `TypeError: 'Future' object is not callable`**
   - Symptom: Error when calling service
   - Cause: Tried to call future object instead of using `call_async()`
   - Solution: Use `future = client.call_async(request)`, then `spin_until_future_complete(node, future)`

3. **Error: Service type mismatch**
   - Symptom: "Service type does not match" error
   - Cause: Client and server using different .srv definitions
   - Solution: Verify both use same interface (e.g., `example_interfaces/srv/AddTwoInts`)

4. **Error: Request fields not set**
   - Symptom: Server receives request but fields are default values (0, empty string)
   - Cause: Forgot to set request fields before calling service
   - Solution: `req.a = 5; req.b = 7` before `client.call_async(req)`

### Citations (Minimum 5, IEEE Format)

Required references:

1. [ROS2Docs2023] Open Robotics, "ROS 2 Documentation: Humble Hawksbill," docs.ros.org, https://docs.ros.org/en/humble/, 2023.
2. [ROS2Services2023] Open Robotics, "About Services," docs.ros.org, https://docs.ros.org/en/humble/Concepts/About-Services.html, 2023.
3. [rclpy2023] Open Robotics, "rclpy - ROS Client Library for Python," GitHub, https://github.com/ros2/rclpy, 2023.
4. [ExampleInterfaces2023] Open Robotics, "example_interfaces Package," GitHub, https://github.com/ros2/example_interfaces, 2023.
5. [ROS2SrvSpec2023] Open Robotics, "Interface Definition using .srv Files," docs.ros.org, https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html, 2023.

## Acceptance Criteria

- [ ] Chapter MDX file exists at `docs/module-1-ros2/02-services.mdx`
- [ ] Word count: 2000-2500 words (±10% acceptable)
- [ ] All 4 code examples present and tested
- [ ] Diagram created (Mermaid or SVG)
- [ ] All 3 exercises with solutions
- [ ] Common Errors section with 4 examples
- [ ] References section with ≥5 IEEE citations
- [ ] Docusaurus build succeeds
- [ ] All code examples run on ROS 2 Humble
- [ ] PEP 8 compliance
- [ ] Validates FR-010 through FR-016
- [ ] Supports SC-002 (student calls service in 15 min)
