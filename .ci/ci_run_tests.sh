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

# Manually export PATH to find mavproxy.py
echo "Current PATH=$PATH"
export PATH=$HOME/.local/bin:$PATH
echo "Updated PATH=$PATH"

# ROS export
source /opt/ros/$ROS_DISTRO/setup.bash

# Catkin export
source ~/$CATKIN_WORKSPACE/devel/setup.bash

# Add sim_vehicle.py to PATH
export PATH=$PATH:$HOME/$CATKIN_WORKSPACE/src/uav_ros_simulation/.gitman/ardupilot/Tools/autotest

# Add libArduPilotPlugin.so to GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$HOME/$CATKIN_WORKSPACE/build/ardupilot_gazebo

# Manually export ccache to PATH
export PATH="/usr/lib/ccache:$PATH"

# Add gcc from boards to PATH
export PATH="/opt/gcc-arm-none-eabi-6-2017-q2-update/bin:$PATH"

# Export master URI
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
echo "ROS_MASTER_URI = $ROS_MASTER_URI"
echo "ROS_HOSTNAME = $ROS_HOSTNAME"

echo "Starting running tests"
cd ~/uav_ws
catkin build uav_ros_tests
catkin build uav_ros_tests --catkin-make-args tests

# folder for test results
TEST_RESULT_PATH=$(realpath /tmp/$RANDOM)
mkdir -p $TEST_RESULT_PATH

# run the test
export UAV_NAMESPACE=red
rostest uav_ros_tests kopterworx_base_rostest.launch -t --results-filename=uav_ros_tests.test --results-base-dir="$TEST_RESULT_PATH"

# evaluate the test results
catkin_test_results "$TEST_RESULT_PATH"
echo "Ended running tests"