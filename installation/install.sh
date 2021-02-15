#!/bin/bash

# Exit immediatelly if a command exits with a non-zero status
set -e

# Executes a command when DEBUG signal is emitted in this script - should be after every line
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# Executes a command when ERR signal is emmitted in this script
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

sudo apt-get -y install git

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

## | --------- change to the directory of this script --------- |

cd "$MY_PATH"

## | --------------------- install ROS ------------------------ |

bash $MY_PATH/dependencies/ros.sh

## | --------------------- install gitman --------------------- |

bash $MY_PATH/dependencies/gitman.sh

## | ---------------- install gitman submodules --------------- |

gitman install --force

# Install uav_ros_stack

bash $MY_PATH/../ros_packages/uav_ros_stack/installation/install.sh

# Install ardupilot

bash $MY_PATH/dependencies/ardupilot_dep.sh
bash $MY_PATH/../firmware/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y

# Add Ardupilot exports to bashrc

num=`cat ~/.bashrc | grep "/ardupilot/Tools/autotest" | wc -l`
if [ "$num" -lt "1" ]; then

  TEMP=`( cd "$MY_PATH/../firmware/ardupilot/Tools/autotest" && pwd )`

  echo "Adding Ardupilot source to .bashrc"
  # set bashrc
  echo "
# Ardupilot exports
export PATH=\$PATH:$TEMP
export PATH=/usr/lib/ccache:\$PATH" >> ~/.bashrc

  if [ -e "$HOME/.zshrc" ]; then
    echo "Adding Ardupilot source to .zshrc"
    echo "
# Ardupilot exports
export PATH=\$PATH:$TEMP
export PATH=/usr/lib/ccache:\$PATH" >> ~/.zshrc
  fi

fi

## | --------------- add ROS sourcing to .bashrc -------------- |

line="source /opt/ros/$ROS_DISTRO/setup.bash"
num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .bashrc"

  # set bashrc
  echo "
$line" >> ~/.bashrc

  if [ -e "$HOME/.zshrc" ]; then
    echo "Adding '$line' to your .zshrc"
    echo "
$line" >> ~/.zshrc
  fi
fi

## | ------------- add Gazebo sourcing to .bashrc ------------- |

line="source /usr/share/gazebo/setup.sh"
num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .bashrc"

  # set bashrc
  echo "
$line" >> ~/.bashrc

  if [ -e "$HOME/.zshrc" ]; then
    echo "Adding '$line' to your .zshrc"
    echo "
$line" >> ~/.zshrc
  fi

fi