#!/bin/bash

# Exit immediatelly if a command exits with a non-zero status
set -e

# Executes a command when DEBUG signal is emitted in this script - should be after every line
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# Executes a command when ERR signal is emmitted in this script
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "$0: installing Ardupilot Dependencies"

if [ "$distro" = "18.04" ]; then
  #  Toppra dependencies
  pip install wheel 
  pip install --upgrade pip

elif [ "$distro" = "20.04" ]; then
  sudo apt -y install python3-pip
  pip3 install wheel
  pip3 install --upgrade pip
fi


sudo apt -y install \
 libgeographic-dev\
 ros-$ROS_DISTRO-mavlink\
 ros-$ROS_DISTRO-mavros\
 ros-$ROS_DISTRO-mavros-msgs\
 ros-$ROS_DISTRO-octomap-ros\
 ros-$ROS_DISTRO-joy\
 protobuf-compiler\
 libgoogle-glog-dev\
 geographiclib-doc\
 geographiclib-tools\
 node-geographiclib\
 screen

if [ "$distro" = "18.04" ]; then
  sudo apt -y install \
 python-wstool\
 libgeographic17

  # TODO: This not needed
  # sudo ./opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh
fi
