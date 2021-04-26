#!/bin/bash

# Setup environment to make ardupilot_gazebo plugins visible to Gazebo.

if [ "$#" != 1 ]; then
    echo -e "usage: ./setup_gazebo.sh /path/to/catkin_ws/build\n"
    exit 1
fi

BUILD_DIR=$1

SNAME=$( echo "$SHELL" | grep -Eo '[^/]+/?$' )
BASHRC=~/.$(echo $SNAME)rc

# setup Gazebo env and update package path
line="export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:${BUILD_DIR}/ardupilot_gazebo"
num=`cat $BASHRC | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then
  echo "Adding '$line' to your $BASHRC"
  echo "# ardupilot_gazebo exports
$line" >> $BASHRC
fi
