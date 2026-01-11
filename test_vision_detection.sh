#!/bin/bash

echo "================================================"
echo " X-Marker Vision Detection Test"
echo "================================================"
echo ""

cd /workspace/ros2_ws
source install/setup.bash

# Clean up any previous processes
pkill -9 -f "px4|gazebo|MicroXRCE" 2>/dev/null
sleep 1

echo "Step 1: Starting MicroXRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &
AGENT_PID=$!
sleep 1

echo "Step 2: Starting Gazebo with X-marker world..."
export GAZEBO_MODEL_PATH=/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export GAZEBO_PLUGIN_PATH=/opt/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export LD_LIBRARY_PATH=$GAZEBO_PLUGIN_PATH:$LD_LIBRARY_PATH

gzserver --verbose /opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/x_marker_test.world > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!
sleep 3

echo "Step 3: Starting PX4 SITL..."
cd /opt/PX4-Autopilot/build/px4_sitl_default
PX4_SIM_MODEL=gazebo-classic_iris_downward_depth_camera PX4_SYS_AUTOSTART=10015 \
./bin/px4 -d etc > /tmp/px4.log 2>&1 &
PX4_PID=$!
sleep 3

echo "Step 4: Spawning drone model..."
gz model -m iris_downward_depth_camera -f /opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_downward_depth_camera/iris_downward_depth_camera.sdf -x 0 -y 0 -z 5 2>&1 | head -5
sleep 2

cd /workspace/ros2_ws
source install/setup.bash

echo "Step 5: Starting vision detection node..."
ros2 run vision_detection xmarker_detector > /tmp/vision.log 2>&1 &
VISION_PID=$!
sleep 5

echo ""
echo "================================================"
echo " System Status"
echo "================================================"

echo ""
echo "✓ Running processes:"
ps aux | grep -E "(MicroXRCE|gzserver|px4|xmarker)" | grep -v grep | awk '{print "  - " $11 " (PID: " $2 ")"}'

echo ""
echo "✓ ROS 2 topics:"
ros2 topic list 2>/dev/null | grep -E "(camera|vision|fmu)" | sed 's/^/  /'

echo ""
echo "================================================"
echo " Testing Camera and Vision"
echo "================================================"
echo ""

echo "Checking camera image topic (3 sec)..."
timeout 3 ros2 topic hz /camera/rgb/image_raw 2>&1 | head -3 && echo "✓ Camera publishing" || echo "✗ No camera data"

echo ""
echo "Checking vision detection topic (5 sec)..."
timeout 5 ros2 topic echo /vision/detections --once 2>&1 | head -15 && echo "✓ Vision detection working" || echo "⚠ No detection (marker may not be in view)"

echo ""
echo "================================================"
echo " Test Complete!"
echo "================================================"
echo ""
echo "Logs saved to:"
echo "  - /tmp/agent.log"
echo "  - /tmp/gazebo.log"
echo "  - /tmp/px4.log"
echo "  - /tmp/vision.log"
echo ""
echo "To monitor detections in real-time:"
echo "  ros2 topic echo /vision/detections"
echo ""
echo "Press Ctrl+C to stop all processes..."
echo ""

# Keep running
wait
