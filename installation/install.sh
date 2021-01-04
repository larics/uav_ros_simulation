#!/bin/bash

# Exit immediatelly if a command exits with a non-zero status
set -e

# Executes a command when DEBUG signal is emitted in this script - should be after every line
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# Executes a command when ERR signal is emmitted in this script
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

sudo apt-get update -qq
DEBIAN_FRONTEND=noninteractive sudo apt-get install -y --no-install-recommends tzdata
sudo apt-get install -y --no-install-recommends dialog apt-utils
sudo apt-get -y install gnupg2 libterm-readline-gnu-perl lsb-release

sudo apt-get update -qq

# the "gce-compute-image-packages" package often freezes the installation at some point
# the installation freezes when it tries to manage some systemd services
# this attempts to install the package and stop the problematic service during the process
((sleep 90 && (sudo systemctl stop google-instance-setup.service && echo "gce service stoped" || echo "gce service not stoped")) & (sudo timeout 120s apt-get -y install gce-compute-image-packages)) || echo "\e[1;31mInstallation of gce-compute-image-packages failed\e[0m"

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"
[ "$distro" = "20.04" ] && DEBIAN_FRONTEND=noninteractive sudo apt-get install -y --no-install-recommends keyboard-configuration

sudo apt-get -y upgrade --fix-missing
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
bash $MY_PATH/../firmware/ardupilot/Tools/install-prereqs-ubuntu.sh

# Add Ardupilot exports to bashrc

num=`cat ~/.bashrc | grep "/ardupilot/Tools/autotest" | wc -l`
if [ "$num" -lt "1" ]; then

  TEMP=`( cd "$MY_PATH/../firmware/ardupilot/Tools/autotest" && pwd )`

  echo "Adding source to .bashrc"
  # set bashrc
  echo "
# Ardupilot exports
export PATH=\$PATH:$TEMP
export PATH=/usr/lib/ccache:\$PATH" >> ~/.bashrc

fi

## | --------------- add ROS sourcing to .bashrc -------------- |

line="source /opt/ros/$ROS_DISTRO/setup.bash"
num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .bashrc"

  # set bashrc
  echo "
$line" >> ~/.bashrc

fi

## | ------------- add Gazebo sourcing to .bashrc ------------- |

line="source /usr/share/gazebo/setup.sh"
num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .bashrc"

  # set bashrc
  echo "
$line" >> ~/.bashrc

fi