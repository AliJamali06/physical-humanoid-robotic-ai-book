# Chapter 3 Contract: Humanoid URDF Basics

**Maps to**: User Story 3 (P3), FR-017 through FR-025
**Target Word Count**: 2500-3000 words
**File**: `Humain-robotic-book/docs/module-1-ros2/03-urdf-basics.mdx`

## Content Requirements

### Code Examples (3 Total - XML URDF Files)

1. **Annotated Simple Humanoid URDF** (4 links, 3 joints)
   - Structure: base_link → torso (fixed joint) → head (revolute joint) + left_arm (revolute joint) + right_arm (revolute joint)
   - OR: base_link/torso → left_arm + right_arm + head (3 children)
   - XML comments explaining each section: `<robot>`, `<link>`, `<joint>`, `<visual>`, `<collision>`, `<inertial>`
   - Includes: link geometry (simple boxes/cylinders), joint limits, joint axes
   - Demonstrates: Basic URDF structure, link-joint hierarchy, tree topology
   - Validates: FR-018, FR-019, FR-020

2. **Joint Limit Modifications**
   - Copy of Example 1 with modified joint limits (e.g., shoulder joint from -π/2 to π/2 changed to -π to π)
   - Inline comments highlighting changed lines
   - Demonstrates: How to adjust joint range for different humanoid designs
   - Validates: FR-023, FR-024 (exercise basis)

3. **Adding a New Link** (simple hand)
   - Extended version of Example 1 with additional link (left_hand, right_hand)
   - New revolute joint connecting arm to hand
   - Demonstrates: How to extend URDF with additional components
   - Validates: FR-024 (exercise basis)

### Diagrams (2 Required)

1. **Link Coordinate Frames**
   - SVG showing simple 2-3 link robot with coordinate frames visualized
   - X=red, Y=green, Z=blue axes (robotics convention)
   - Show origin points, joint axes
   - Labels: frame names (`base_link`, `torso`, `head`)
   - Demonstrates: Spatial relationships, transform tree concept

2. **Link-Joint Tree** (from `urdf_to_graphiz` output)
   - Either: Actual output from running `urdf_to_graphiz simple_humanoid.urdf` (converted to SVG)
   - OR: Hand-drawn tree diagram matching the tool's output format
   - Shows: Parent-child relationships, joint types in boxes
   - Validates: FR-025

### Exercises (3 Total)

1. **Basic: Identify Joint Types in URDF**
   - Objective: Given annotated URDF, list all joint names and their types (revolute, fixed, continuous)
   - Steps: Open Example 1, locate each `<joint>` tag, identify `type` attribute, create table
   - Verification: Self-check against solution table
   - Solution: Table with columns [Joint Name | Type | Parent Link | Child Link]

2. **Intermediate: Modify Joint Limits and Validate**
   - Objective: Change shoulder joint upper limit from π/2 to π, validate with `check_urdf`
   - Steps: Locate joint in URDF, modify `<limit upper="...">`, save, run `check_urdf`
   - Verification: `check_urdf` outputs "Successfully Parsed XML" (validates SC-004)
   - Solution: Changed line with explanation of radians vs degrees

3. **Advanced: Add Simple Hand Link**
   - Objective: Add `left_hand` link connected to `left_arm` with revolute joint (wrist rotation)
   - Steps: Copy link template, define geometry (small box), create joint, set limits, validate
   - Verification: `check_urdf` passes, `urdf_to_graphiz` shows new link in tree
   - Solution: Full XML snippet for new link and joint (10-15 lines)

### Prerequisites List

- ROS 2 Humble Hawksbill installed
- `urdfdom` package: `sudo apt install liburdfdom-tools`
- Graphviz (for `urdf_to_graphiz`): `sudo apt install graphviz`
- Text editor with XML syntax highlighting (VS Code, Sublime Text)
- Basic understanding of 3D coordinate systems (X, Y, Z axes)
- Completion of Chapters 1 and 2 (optional but recommended)

### Learning Objectives (4 Measurable Outcomes)

By the end of this chapter, you will be able to:

1. **Explain the purpose of URDF** in robotics (geometric and kinematic robot description) with humanoid context (5 minutes)
2. **Read a humanoid URDF file** and identify all links, joints, and their parent-child relationships (10 minutes, validates SC-003)
3. **Modify joint limits** in a URDF and validate changes using `check_urdf` without errors (5 minutes, validates SC-004)
4. **Add a simple link** to an existing URDF (e.g., hand, camera) and verify with `urdf_to_graphiz` (15 minutes)

### Common Errors (5 Examples)

1. **Error: `Failed to parse urdf file`**
   - Symptom: `check_urdf` reports parse error
   - Cause: XML syntax error (missing closing tag, unclosed quote, wrong tag name)
   - Solution: Check XML syntax carefully; use editor with XML validation; look for line number in error message

2. **Error: Circular dependency detected**
   - Symptom: `check_urdf` reports "Joint [name] has circular parent-child relationship"
   - Cause: Joint's child link is also an ancestor (creates loop)
   - Solution: Review link tree, ensure strict parent → child hierarchy with no loops

3. **Error: Invalid joint type**
   - Symptom: `check_urdf` reports "Unknown joint type: [type]"
   - Cause: Typo in `<joint type="...">` (e.g., `"Revolute"` instead of `"revolute"`, `"rotational"` instead of `"revolute"`)
   - Solution: Use lowercase exact types: `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`

4. **Error: Missing `<inertial>` tag**
   - Symptom: Warning: "Link [name] has no inertial data"
   - Cause: No `<inertial>` section in link (required for simulation)
   - Solution: Add minimal inertial: `<mass value="1.0"/>`, `<inertia ixx="0.01" .../>` (for basic examples, use placeholder values)

5. **Error: Joint limit upper < lower**
   - Symptom: `check_urdf` may warn or simulation behaves oddly
   - Cause: `<limit lower="1.57" upper="0.0"/>` (upper less than lower)
   - Solution: Ensure `lower ≤ upper` for revolute joints; check radian values (π ≈ 3.14159)

### Citations (Minimum 5, IEEE Format)

Required references:

1. [URDFSpec2023] Open Robotics, "Unified Robot Description Format (URDF)," ROS Wiki, http://wiki.ros.org/urdf/XML, 2023.
2. [ROS2URDF2023] Open Robotics, "URDF Package Documentation," docs.ros.org, https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html, 2023.
3. [urdfdom2023] Open Robotics, "urdfdom - URDF Parser Library," GitHub, https://github.com/ros/urdfdom, 2023.
4. [ROS2TF2023] Open Robotics, "Understanding TF (Transform Library)," docs.ros.org, https://docs.ros.org/en/humble/Concepts/About-Tf2.html, 2023.
5. [HumanoidURDF2020] P. Beeson and B. Ames, "TRAC-IK: An Inverse Kinematics Library for Humanoid Robots," IEEE Robotics and Automation Letters, vol. 1, no. 1, pp. 43-50, 2020. (Example research paper citation)

## Acceptance Criteria

- [ ] Chapter MDX file exists at `docs/module-1-ros2/03-urdf-basics.mdx`
- [ ] Word count: 2500-3000 words (±10% acceptable)
- [ ] All 3 URDF examples created and validated with `check_urdf`
- [ ] Both diagrams created (coordinate frames SVG, link tree from `urdf_to_graphiz` or hand-drawn)
- [ ] All 3 exercises with solutions
- [ ] Common Errors section with 5 examples
- [ ] References section with ≥5 IEEE citations
- [ ] Docusaurus build succeeds
- [ ] All URDF files valid XML and pass `check_urdf`
- [ ] Validates FR-017 through FR-025
- [ ] Supports SC-003 and SC-004 (URDF reading/modification)
