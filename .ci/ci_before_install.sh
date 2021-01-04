#!/bin/bash
  
./installation/install.sh
./ros_packages/uav_ros_stack/installation/workspace_setup.sh

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

cd ~/uav_ws/src
ln -s $GITHUB_WORKSPACE
source /opt/ros/$ROS_DISTRO/setup.bash