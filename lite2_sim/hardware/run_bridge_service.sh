#!/bin/bash
# This launcher never starts/restarts the motion firmware or requests posture.
set -e
source /opt/ros/melodic/setup.bash
set -u
deployment_root="${AIRACE_DEPLOYMENT_ROOT:-/home/ysc/airace_deployment_20261114}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-192.168.1.103}"
cd "$deployment_root/hardware"
exec python3 -u lite2_live.py bridge
