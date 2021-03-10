#!/bin/bash

# This script is used to unpack uav_ros_simulation packages in a development-friendly way
# The end result of this script should be as follows:
# catkin_ws
# - src
# --- uav_ros_simulation (with dirty git + CATKIN_IGNORE)
# --- uav_ros_simulation_modules
# ------ uav_ros_stack (with dirty git + CATKIN_IGNORE)
# ------ ardupilot_gazebo
# ------ mav_comm
# ------ ardupilot
# --- uav_ros_stack_modules
# ------ uav_ros_control
# ------ uav_ros_lib
# ------ uav_ros_tests
# ------ uav_ros_msgs
# ------ uav_ros_general
# ------ topp_ros

# Exit immediatelly if a command exits with a non-zero status
set -e

# Executes a command when DEBUG signal is emitted in this script - should be after every line
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# Executes a command when ERR signal is emmitted in this script
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd $MY_PATH/..

echo "Uninstalling gitman modules in $(pwd)"
gitman uninstall -f

echo "Stashing git changes in $(pwd)"
git stash

echo "Updating gitman.yml with new location"
sed -i '1s@.*@location: ../uav_ros_simulation_modules@' gitman.yml
gitman update -f
touch "CATKIN_IGNORE"

cd ros_packages/uav_ros_stack
echo "Uninstalling gitman modules in $(pwd)"
gitman uninstall -f

echo "Stashing git changes in $(pwd)"
git stash
echo "Updating gitman.yml with new location"
sed -i '1s@.*@location: ../../uav_ros_stack_modules@' gitman.yml
gitman update -f
touch "CATKIN_IGNORE"

cd $MY_PATH