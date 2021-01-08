#!/bin/bash

# Setup environment to make ardupilot_gazebo plugins visible to Gazebo.

if [ "$#" != 1 ]; then
    echo -e "usage: ./setup_gazebo.sh /path/to/catkin_ws/build\n"
    return 1
fi

BUILD_DIR=$1

# setup Gazebo env and update package path
line="export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:${BUILD_DIR}/ardupilot_gazebo"
num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .bashrc"

  # set bashrc
  echo "
# ardupilot_gazebo exports
$line" >> ~/.bashrc

  if [ -e "$HOME/.zshrc" ]; then
    echo "Adding '$line' to your .zshrc"
    echo "
# ardupilot_gazebo exports
$line" >> ~/.zshrc
  fi

fi
