#!/bin/bash

# ROS2 Humble
source /opt/ros/humble/setup.bash

# px4_msgs workspace
source /opt/px4_msgs_ws/install/setup.bash

# Current workspace
source /workspace/ros2_ws/install/setup.bash

# PX4 paths
export PX4_DIR=/opt/PX4-Autopilot
export PX4_BUILD_DIR=$PX4_DIR/build/px4_sitl_default

# Gazebo environment
export GAZEBO_MODEL_PATH=$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$PX4_BUILD_DIR/build_gazebo-classic:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=$PX4_BUILD_DIR/build_gazebo-classic:$LD_LIBRARY_PATH
export GAZEBO_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$GAZEBO_RESOURCE_PATH

# ROS2 for Gazebo
export ROS_VERSION=2

# X11 Display for GUI (Guacamole environment)
export DISPLAY=:1.0

echo "========================================="
echo "PX4 + Gazebo Classic + ROS2 Environment"
echo "========================================="
echo "PX4 Directory: $PX4_DIR"
echo "Gazebo Model Path: $GAZEBO_MODEL_PATH"
echo "ROS2 Distro: $ROS_DISTRO"
echo "========================================="
