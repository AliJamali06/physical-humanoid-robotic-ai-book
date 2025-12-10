#!/bin/bash

################################################################################
# Nav2 Navigation Launch Commands
#
# This script provides example ROS 2 commands for launching and testing
# Nav2 autonomous navigation with various configurations.
#
# Prerequisites:
# - ROS 2 Humble installed and sourced
# - Nav2 installed: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
# - Localization active (VSLAM or AMCL)
# - Map available (for AMCL) or VSLAM building map on-the-fly
#
# Usage:
#   chmod +x launch_nav2.sh
#   ./launch_nav2.sh
#
# Or run individual commands by copying them to your terminal.
################################################################################

# Source ROS 2 environment (adjust path if needed)
source /opt/ros/humble/setup.bash

echo "=========================================="
echo "Nav2 Navigation Launch Examples"
echo "=========================================="

# ==============================================================================
# EXAMPLE 1: Basic Nav2 Launch (Default Configuration)
# ==============================================================================

echo ""
echo "[1] Basic Nav2 Launch with Default Parameters"
echo "Command:"
echo "  ros2 launch nav2_bringup navigation_launch.py"
echo ""
echo "Expected behavior:"
echo "  - Starts global planner (A*)"
echo "  - Starts local planner (DWB)"
echo "  - Starts cost map servers (global + local)"
echo "  - Subscribes to /map and sensor topics"
echo "  - Listens for navigation goals on /goal_pose"
echo ""

# Uncomment to run:
# ros2 launch nav2_bringup navigation_launch.py

# ==============================================================================
# EXAMPLE 2: Nav2 with Custom Parameters File
# ==============================================================================

echo "[2] Nav2 with Custom YAML Configuration"
echo "Command:"
echo "  ros2 launch nav2_bringup navigation_launch.py \\"
echo "    params_file:=/path/to/nav2_params.yaml"
echo ""
echo "Note: Combine nav2_global_planner.yaml and nav2_local_planner.yaml"
echo "      into a single nav2_params.yaml file"
echo ""

# Uncomment to run (adjust path):
# ros2 launch nav2_bringup navigation_launch.py \
#   params_file:=./nav2_params.yaml

# ==============================================================================
# EXAMPLE 3: Nav2 with Simulation Time (Isaac Sim or Gazebo)
# ==============================================================================

echo "[3] Nav2 with Simulation Time"
echo "Command:"
echo "  ros2 launch nav2_bringup navigation_launch.py \\"
echo "    use_sim_time:=true"
echo ""
echo "Use case: When running with Isaac Sim or Gazebo simulator"
echo ""

# Uncomment to run:
# ros2 launch nav2_bringup navigation_launch.py \
#   use_sim_time:=true

# ==============================================================================
# EXAMPLE 4: Load Map (AMCL Localization)
# ==============================================================================

echo "[4] Load Pre-Built Map for AMCL"
echo "Command:"
echo "  ros2 launch nav2_bringup localization_launch.py \\"
echo "    map:=/path/to/map.yaml \\"
echo "    use_sim_time:=true"
echo ""
echo "Note: Skip this if using VSLAM (VSLAM builds map on-the-fly)"
echo ""

# Uncomment to run:
# ros2 launch nav2_bringup localization_launch.py \
#   map:=./my_map.yaml \
#   use_sim_time:=true

# ==============================================================================
# EXAMPLE 5: Send Navigation Goal (Command Line)
# ==============================================================================

echo "[5] Send Navigation Goal via Topic"
echo "Command:"
echo "  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \\"
echo "    '{header: {frame_id: \"map\"}, "
echo "      pose: {position: {x: 5.0, y: 3.0, z: 0.0}, "
echo "             orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}'"
echo ""
echo "This sends robot to position (5.0, 3.0) with 45° orientation"
echo ""

# Uncomment to run:
# ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
#   '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}'

# ==============================================================================
# EXAMPLE 6: Send Navigation Goal via Action (Python Script)
# ==============================================================================

echo "[6] Send Navigation Goal via Action Client (Python)"
echo "Create a Python script:"
cat << 'PYTHON_SCRIPT'
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

def send_goal():
    rclpy.init()
    node = rclpy.create_node('nav2_goal_client')
    action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # Wait for action server
    action_client.wait_for_server()

    # Create goal message
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.pose.position.x = 5.0
    goal_msg.pose.pose.position.y = 3.0
    goal_msg.pose.pose.orientation.w = 1.0

    # Send goal
    print("Sending navigation goal...")
    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)

    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        print("Goal rejected!")
        return

    print("Goal accepted! Navigating...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    print("Navigation complete!")

if __name__ == '__main__':
    send_goal()
PYTHON_SCRIPT

echo ""
echo "Save as nav2_goal.py and run: python3 nav2_goal.py"
echo ""

# ==============================================================================
# EXAMPLE 7: Monitor Global Plan
# ==============================================================================

echo "[7] Monitor Global Plan (Full Path to Goal)"
echo "Command:"
echo "  ros2 topic echo /plan"
echo ""
echo "Expected output (nav_msgs/Path):"
echo "  header:"
echo "    frame_id: 'map'"
echo "  poses:"
echo "    - pose:"
echo "        position: {x: 0.0, y: 0.0, z: 0.0}  # Start"
echo "    - pose:"
echo "        position: {x: 1.0, y: 0.5, z: 0.0}  # Waypoint 1"
echo "    ..."
echo "    - pose:"
echo "        position: {x: 5.0, y: 3.0, z: 0.0}  # Goal"
echo ""

# Uncomment to run:
# ros2 topic echo /plan

# ==============================================================================
# EXAMPLE 8: Monitor Local Plan (Dynamic Trajectory)
# ==============================================================================

echo "[8] Monitor Local Plan (Immediate Trajectory)"
echo "Command:"
echo "  ros2 topic echo /local_plan"
echo ""
echo "Shows short-term trajectory (next 1-3 seconds)"
echo "Updated at controller frequency (10-20 Hz)"
echo ""

# Uncomment to run:
# ros2 topic echo /local_plan

# ==============================================================================
# EXAMPLE 9: Monitor Velocity Commands
# ==============================================================================

echo "[9] Monitor Velocity Commands Sent to Robot"
echo "Command:"
echo "  ros2 topic echo /cmd_vel"
echo ""
echo "Expected output (geometry_msgs/Twist):"
echo "  linear:"
echo "    x: 0.5  # Forward velocity (m/s)"
echo "    y: 0.0"
echo "    z: 0.0"
echo "  angular:"
echo "    x: 0.0"
echo "    y: 0.0"
echo "    z: 0.15  # Rotation (rad/s, ~8.6°/s)"
echo ""

# Uncomment to run:
# ros2 topic echo /cmd_vel

# ==============================================================================
# EXAMPLE 10: Visualize Cost Maps in RViz
# ==============================================================================

echo "[10] Visualize Nav2 in RViz"
echo "Command:"
echo "  ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"
echo ""
echo "RViz will display:"
echo "  - Global cost map (static obstacles)"
echo "  - Local cost map (dynamic obstacles)"
echo "  - Global plan (yellow/green path)"
echo "  - Local plan (red/orange trajectory)"
echo "  - Robot footprint"
echo "  - Sensor data (LiDAR, camera)"
echo ""

# Uncomment to run:
# ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

# ==============================================================================
# EXAMPLE 11: Cancel Current Navigation Goal
# ==============================================================================

echo "[11] Cancel Active Navigation Goal"
echo "Command:"
echo "  ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose '{}' --cancel"
echo ""
echo "Or simply publish an empty goal:"
echo "  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{}'"
echo ""

# Uncomment to run:
# ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{}'

# ==============================================================================
# EXAMPLE 12: Clear Cost Map (Recovery Behavior)
# ==============================================================================

echo "[12] Clear Cost Maps (Troubleshooting)"
echo "Commands:"
echo "  # Clear global cost map:"
echo "  ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty"
echo ""
echo "  # Clear local cost map:"
echo "  ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty"
echo ""
echo "Use case: Robot thinks space is occupied when it's actually clear"
echo ""

# Uncomment to run:
# ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty
# ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty

# ==============================================================================
# EXAMPLE 13: Check Nav2 Lifecycle Node States
# ==============================================================================

echo "[13] Check Nav2 Node States"
echo "Command:"
echo "  ros2 lifecycle list /controller_server"
echo "  ros2 lifecycle list /planner_server"
echo "  ros2 lifecycle list /bt_navigator"
echo ""
echo "Expected output: 'active' (nodes are running)"
echo ""

# Uncomment to run:
# ros2 lifecycle list /controller_server

# ==============================================================================
# EXAMPLE 14: Full Pipeline Integration (Isaac Sim + VSLAM + Nav2)
# ==============================================================================

echo "[14] Complete Navigation Pipeline"
echo ""
echo "Terminal 1 - Launch Isaac Sim:"
echo "  (Start Isaac Sim GUI, load warehouse scene with humanoid robot)"
echo ""
echo "Terminal 2 - Launch VSLAM:"
echo "  ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py"
echo ""
echo "Terminal 3 - Launch Nav2:"
echo "  ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true"
echo ""
echo "Terminal 4 - Launch RViz:"
echo "  ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"
echo ""
echo "Terminal 5 - Send Navigation Goal:"
echo "  (Use '2D Nav Goal' button in RViz or publish to /goal_pose)"
echo ""

# ==============================================================================
# EXAMPLE 15: Monitor Nav2 Performance Metrics
# ==============================================================================

echo "[15] Monitor Nav2 Performance"
echo "Commands:"
echo "  # Check controller update rate:"
echo "  ros2 topic hz /cmd_vel"
echo ""
echo "  # Check planner update rate:"
echo "  ros2 topic hz /plan"
echo ""
echo "  # Check cost map update rate:"
echo "  ros2 topic hz /local_costmap/costmap"
echo ""
echo "Expected rates:"
echo "  - /cmd_vel: 10-20 Hz (controller frequency)"
echo "  - /plan: 0.5-2 Hz (replanning frequency)"
echo "  - /local_costmap: 5-10 Hz (sensor update rate)"
echo ""

# Uncomment to run:
# ros2 topic hz /cmd_vel

# ==============================================================================
# Troubleshooting Tips
# ==============================================================================

echo ""
echo "=========================================="
echo "Troubleshooting Common Nav2 Issues"
echo "=========================================="
echo ""

echo "Issue 1: No global plan generated"
echo "Solution: Check if localization is active:"
echo "  ros2 topic hz /tf  # Should show map -> odom transform"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo ""

echo "Issue 2: Robot oscillates (back and forth)"
echo "Solution: Tune DWB cost weights:"
echo "  Increase PathAlign.scale (stick to path)"
echo "  Reduce inflation_radius (less conservative)"
echo ""

echo "Issue 3: Robot ignores obstacles"
echo "Solution: Check obstacle layer receiving data:"
echo "  ros2 topic hz /scan  # Should be publishing"
echo "  ros2 topic echo /local_costmap/costmap --once  # Check for occupied cells"
echo ""

echo "Issue 4: Path goes through walls"
echo "Solution: Verify map is loaded:"
echo "  ros2 topic echo /map --once"
echo "  Check that static_layer is enabled in cost map config"
echo ""

echo "Issue 5: Robot gets stuck"
echo "Solution: Clear cost maps and retry:"
echo "  ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty"
echo "  Increase goal tolerance (xy_goal_tolerance: 0.3)"
echo ""

# ==============================================================================
# Cleanup and Exit
# ==============================================================================

echo ""
echo "=========================================="
echo "Script complete. Commands are ready to copy."
echo "Uncomment specific examples to run them."
echo "=========================================="
echo ""

# To kill all Nav2 nodes (cleanup):
# ros2 lifecycle set /controller_server shutdown
# ros2 lifecycle set /planner_server shutdown
# ros2 lifecycle set /bt_navigator shutdown
# killall -9 planner_server controller_server bt_navigator
