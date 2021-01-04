#!/bin/bash

set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting test build"
cd ~/uav_ws/src
source /opt/ros/$ROS_DISTRO/setup.bash
catkin build
echo "Ended test build"