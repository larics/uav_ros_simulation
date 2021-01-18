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

# Install again just in case
bash $HOME/uav_ws/src/uav_ros_simulation/.gitman/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y

# Manually export PYTHONPATH
echo "Current PYTHONPATH=$PYTHONPATH"
echo "Current PATH=$PATH"
if [ "$distro" = "18.04" ]; then
  export PYTHONPATH=$HOME/.local/lib/python2.7/site-packages/MAVProxy:$PYTHONPATH
  export PYTHONPATH=/usr/local/lib/python2.7/site-packages/MAVProxy:$PYTHONPATH
  export PATH=$HOME/.local/lib/python2.7/site-packages/MAVProxy:$PATH
  export PATH=/usr/local/lib/python2.7/site-packages/MAVProxy:$PATH
elif [ "$distro" = "20.04" ]; then
  export PYTHONPATH=$HOME/.local/lib/python3.8/site-packages/MAVProxy:$PYTHONPATH
  export PYTHONPATH=/usr/local/lib/python3.8/site-packages/MAVProxy:$PYTHONPATH
  export PATH=$HOME/.local/lib/python3.8/site-packages/MAVProxy:$PATH
  export PATH=/usr/local/lib/python3.8/site-packages/MAVProxy:$PATH
fi
echo "Updated PATH=$PATH"
echo "Updated PYTHONPATH=$PYTHONPATH"

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/uav_ws/devel/setup.bash

# Add sim_vehicle.py to PATH
export PATH=$PATH:$HOME/uav_ws/src/uav_ros_simulation/.gitman/ardupilot/Tools/autotest

# Add libArduPilotPlugin.so to PATH
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$HOME/uav_ws/build/ardupilot_gazebo

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