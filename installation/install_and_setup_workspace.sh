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

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

## | --------- change to the directory of this script --------- |
cd "$MY_PATH"

WORKSPACE_NAME=uav_ws
SPARSE=""
while true; do
  case "$1" in
  --workspace ) WORKSPACE_NAME=$2; shift 2 ;;
  --sparse ) SPARSE="--sparse"; shift ;;
  * ) break ;;
  esac
done

# Install uav_ros_simulation
bash $MY_PATH/install.sh $SPARSE

# Setup catkin workspace
bash $MY_PATH/../ros_packages/uav_ros_stack/installation/workspace_setup.sh $WORKSPACE_NAME

# Build catkin workspace
ROOT_DIR=`dirname $MY_PATH`
cd ~/$WORKSPACE_NAME/src
ln -s $ROOT_DIR
source /opt/ros/$ROS_DISTRO/setup.bash

# Setup Gazebo
bash $MY_PATH/gazebo/setup_gazebo.sh $HOME/$WORKSPACE_NAME/build
