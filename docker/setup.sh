#!/usr/bin/env bash
set -e

### ROS2 WORKSPACE SETUP #######################################
mkdir -p /root/dev_ws/src

# Link the whole folder instead of wildcard contents
if [ -d "/root/repo/ros2_ws" ]; then
    ln -sfn /root/repo/ros2_ws/ /root/dev_ws/src/
fi

### PYTORCH / ML WORKSPACE SETUP ################################
mkdir -p /root/ml_ws/src

if [ -d "/root/repo/vlm" ]; then
    ln -sfn /root/repo/vlm/ /root/ml_ws/
fi