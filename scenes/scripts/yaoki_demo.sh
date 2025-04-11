#!/bin/bash

# Run joint command
echo "Publishing initial joint command..."
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{name: ['RevoluteJoint'], position: [1]}"

# Wait a moment for the joint command to be processed
sleep 1

# Launch teleop
echo "Starting ROS2 teleop..."
ros2 run teleop_twist_keyboard teleop_twist_keyboard
