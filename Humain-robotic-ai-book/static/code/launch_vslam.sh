#!/bin/bash

################################################################################
# Isaac ROS Visual SLAM Launch Commands
#
# This script provides example ROS 2 commands for launching and testing
# Isaac ROS Visual SLAM with various configurations.
#
# Prerequisites:
# - ROS 2 Humble installed and sourced
# - Isaac ROS VSLAM installed: sudo apt install ros-humble-isaac-ros-visual-slam
# - Camera node running (publishing to /camera/image_raw and /camera/camera_info)
#
# Usage:
#   chmod +x launch_vslam.sh
#   ./launch_vslam.sh
#
# Or run individual commands by copying them to your terminal.
################################################################################

# Source ROS 2 environment (adjust path if needed)
source /opt/ros/humble/setup.bash

echo "=========================================="
echo "Isaac ROS Visual SLAM Launch Examples"
echo "=========================================="

# ==============================================================================
# EXAMPLE 1: Basic VSLAM Launch (Default Configuration)
# ==============================================================================

echo ""
echo "[1] Basic VSLAM Launch with Default Parameters"
echo "Command:"
echo "  ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py"
echo ""
echo "Expected behavior:"
echo "  - Subscribes to /camera/image_raw and /camera/camera_info"
echo "  - Publishes odometry to /visual_slam/tracking/odometry"
echo "  - Publishes TF: map -> odom"
echo "  - Default: 1000 features, loop closure enabled, GPU acceleration ON"
echo ""

# Uncomment to run:
# ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# ==============================================================================
# EXAMPLE 2: VSLAM with Custom Configuration File
# ==============================================================================

echo "[2] VSLAM with Custom YAML Configuration"
echo "Command:"
echo "  ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \\"
echo "    config_file:=/path/to/vslam_config.yaml"
echo ""
echo "Note: Replace /path/to/vslam_config.yaml with actual path"
echo ""

# Uncomment to run (adjust path):
# ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
#   config_file:=./vslam_config.yaml

# ==============================================================================
# EXAMPLE 3: VSLAM with Remapped Camera Topics
# ==============================================================================

echo "[3] VSLAM with Remapped Camera Topics"
echo "Command:"
echo "  ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \\"
echo "    camera_image_topic:=/my_robot/camera/image \\"
echo "    camera_info_topic:=/my_robot/camera/info"
echo ""
echo "Use case: When camera topics have non-standard names"
echo ""

# Uncomment to run:
# ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
#   camera_image_topic:=/my_robot/camera/image \
#   camera_info_topic:=/my_robot/camera/info

# ==============================================================================
# EXAMPLE 4: Monitor VSLAM Output Topics
# ==============================================================================

echo "[4] Monitor VSLAM Odometry Output"
echo "Command (in separate terminal):"
echo "  ros2 topic echo /visual_slam/tracking/odometry"
echo ""
echo "Expected output (nav_msgs/Odometry):"
echo "  header:"
echo "    frame_id: 'map'"
echo "  child_frame_id: 'odom'"
echo "  pose:"
echo "    position: {x: 1.23, y: 0.45, z: 0.0}"
echo "    orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}"
echo "  twist:"
echo "    linear: {x: 0.5, y: 0.0, z: 0.0}"
echo "    angular: {x: 0.0, y: 0.0, z: 0.2}"
echo ""

# Uncomment to run:
# ros2 topic echo /visual_slam/tracking/odometry

# ==============================================================================
# EXAMPLE 5: Visualize VSLAM Trajectory in RViz
# ==============================================================================

echo "[5] Visualize VSLAM in RViz"
echo "Command:"
echo "  ros2 run rviz2 rviz2 -d vslam_visualization.rviz"
echo ""
echo "RViz Configuration:"
echo "  - Add 'Path' display: /visual_slam/vis/slam_path"
echo "  - Add 'TF' display: show map, odom, base_link, camera_link frames"
echo "  - Add 'Odometry' display: /visual_slam/tracking/odometry"
echo "  - Set Fixed Frame: 'map'"
echo ""

# Uncomment to run:
# ros2 run rviz2 rviz2

# ==============================================================================
# EXAMPLE 6: Check TF Transform (map -> odom)
# ==============================================================================

echo "[6] Verify TF Transform (map -> odom)"
echo "Command:"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo ""
echo "Expected output:"
echo "  At time [timestamp]"
echo "  - Translation: [1.234, 0.567, 0.000]"
echo "  - Rotation: in Quaternion [0.000, 0.000, 0.383, 0.924]"
echo "            in RPY (radian) [0.000, 0.000, 0.785]"
echo "            in RPY (degree) [0.000, 0.000, 45.000]"
echo ""

# Uncomment to run:
# ros2 run tf2_ros tf2_echo map odom

# ==============================================================================
# EXAMPLE 7: List All VSLAM Topics
# ==============================================================================

echo "[7] List All VSLAM-Related Topics"
echo "Command:"
echo "  ros2 topic list | grep visual_slam"
echo ""
echo "Expected topics:"
echo "  /visual_slam/tracking/odometry"
echo "  /visual_slam/tracking/vo_pose"
echo "  /visual_slam/vis/slam_path"
echo "  /visual_slam/diagnostics"
echo ""

# Uncomment to run:
# ros2 topic list | grep visual_slam

# ==============================================================================
# EXAMPLE 8: Check VSLAM Node Parameters
# ==============================================================================

echo "[8] Inspect VSLAM Node Parameters"
echo "Command:"
echo "  ros2 param list /visual_slam_node"
echo ""
echo "Shows all configurable parameters (num_features, enable_loop_closure, etc.)"
echo ""

# Uncomment to run:
# ros2 param list /visual_slam_node

# ==============================================================================
# EXAMPLE 9: Record VSLAM Data to ROS Bag
# ==============================================================================

echo "[9] Record VSLAM Session to ROS 2 Bag"
echo "Command:"
echo "  ros2 bag record -o vslam_session \\"
echo "    /camera/image_raw \\"
echo "    /camera/camera_info \\"
echo "    /visual_slam/tracking/odometry \\"
echo "    /visual_slam/vis/slam_path \\"
echo "    /tf \\"
echo "    /tf_static"
echo ""
echo "Use case: Record data for offline analysis or replay"
echo ""

# Uncomment to run:
# ros2 bag record -o vslam_session \
#   /camera/image_raw \
#   /camera/camera_info \
#   /visual_slam/tracking/odometry \
#   /visual_slam/vis/slam_path \
#   /tf \
#   /tf_static

# ==============================================================================
# EXAMPLE 10: Replay Recorded VSLAM Session
# ==============================================================================

echo "[10] Replay Recorded ROS 2 Bag"
echo "Command:"
echo "  ros2 bag play vslam_session"
echo ""
echo "Note: Launch VSLAM node first, then replay bag"
echo "VSLAM will reprocess recorded camera images"
echo ""

# Uncomment to run:
# ros2 bag play vslam_session

# ==============================================================================
# EXAMPLE 11: Integration with Isaac Sim
# ==============================================================================

echo "[11] Full Pipeline: Isaac Sim + VSLAM + RViz"
echo ""
echo "Terminal 1 - Launch Isaac Sim:"
echo "  (Start Isaac Sim GUI and load scene with robot + camera)"
echo ""
echo "Terminal 2 - Launch VSLAM:"
echo "  ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py"
echo ""
echo "Terminal 3 - Launch RViz:"
echo "  ros2 run rviz2 rviz2"
echo ""
echo "Terminal 4 - Drive Robot (optional):"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""

# ==============================================================================
# EXAMPLE 12: Check VSLAM Performance Metrics
# ==============================================================================

echo "[12] Monitor VSLAM Performance"
echo "Command:"
echo "  ros2 topic hz /visual_slam/tracking/odometry"
echo ""
echo "Expected output:"
echo "  average rate: 50-100 Hz (GPU acceleration)"
echo "  average rate: 10-30 Hz (CPU fallback)"
echo ""

# Uncomment to run:
# ros2 topic hz /visual_slam/tracking/odometry

# ==============================================================================
# Troubleshooting Tips
# ==============================================================================

echo ""
echo "=========================================="
echo "Troubleshooting Common Issues"
echo "=========================================="
echo ""

echo "Issue 1: No odometry published"
echo "Solution: Check camera topics are active:"
echo "  ros2 topic list | grep camera"
echo "  ros2 topic hz /camera/image_raw"
echo ""

echo "Issue 2: Low update rate (<10 Hz)"
echo "Solution: Verify GPU acceleration enabled:"
echo "  ros2 param get /visual_slam_node use_gpu"
echo "  (should return: True)"
echo ""

echo "Issue 3: VSLAM crashes or produces NaN values"
echo "Solution: Check camera_info is valid:"
echo "  ros2 topic echo /camera/camera_info --once"
echo "  (verify K matrix has non-zero focal length)"
echo ""

echo "Issue 4: High drift (position error accumulates)"
echo "Solution: Enable loop closure:"
echo "  ros2 param set /visual_slam_node enable_loop_closure true"
echo ""

echo "Issue 5: No features detected (low-texture environment)"
echo "Solution: Increase feature count:"
echo "  ros2 param set /visual_slam_node num_features 1500"
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

# To kill all ROS 2 nodes (cleanup):
# killall -9 visual_slam_node
# killall -9 rviz2
