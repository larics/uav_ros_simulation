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


# Executes a command when DEBUG signal is emitted in this script - should be after every line
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# Executes a command when ERR signal is emmitted in this script
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

if [ "$#" -ne 1 ]; then
    echo "One argument expected - path to the 'src' folder of the Catkin Workspace"
    exit 2
fi

CATKIN_SRC=$1

# Check if this is a directory
if [[ ! -d "$CATKIN_SRC" ]]; then
    echo "$CATKIN_SRC is not a directory"
    exit 2
fi

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

SIM_PATH=$(readlink -f "$MY_PATH/../../uav_ros_simulation")
SIM_MODULES_PATH=$(readlink -f "$MY_PATH/../../uav_ros_simulation_modules")
STACK_MODULES_PATH=$(readlink -f "$MY_PATH/../../uav_ros_stack_modules")

# Link all the modules
ln -s $SIM_PATH $CATKIN_SRC
ln -s $SIM_MODULES_PATH $CATKIN_SRC
ln -s $STACK_MODULES_PATH $CATKIN_SRC

# Link the development tools
ln -s $SIM_PATH/ros_packages/uav_ros_stack/.clang-format $CATKIN_SRC/.clang-format
ln -s $SIM_PATH/ros_packages/uav_ros_stack/.cmake-format.yaml $CATKIN_SRC/.cmake-format.yaml

cd $MY_PATH