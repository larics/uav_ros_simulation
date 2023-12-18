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

sudo apt-get -y install git

# Python fix
if [ "$distro" = "20.04" ]; then
  sudo ln -sf /usr/bin/python3 /usr/bin/python
fi

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

# Check if we're installing uav_ros_stackk full (stack) or only partial (stack_sparse)
WHICH_STACK="stack"
while true; do
  case "$1" in
  --sparse ) WHICH_STACK="stack_sparse"; shift ;;
  * ) break ;;
  esac
done

# Check if rotors-gazebo is already installed via apt
sudo apt-get -y install dpkg
PACKAGE=ros-$ROS_DISTRO-rotors-gazebo
num=`dpkg --list | grep $PACKAGE | wc -l`
if [ "$num" -gt "0" ]; then
  echo -e "\e[1;33m WARNING: You already have $PACKAGE installed with apt. \
Please uninstall it by running sudo apt-get purge $PACKAGE. The uav_ros_simulation \
would like to build it from source at \
https://github.com/larics/rotors_simulator/tree/larics_master. \e0"
  exit 2
fi

# Check if mav-msgs is already installed via apt
PACKAGE=ros-$ROS_DISTRO-mav-msgs
num=`dpkg --list | grep $PACKAGE | wc -l`
if [ "$num" -gt "0" ]; then
  echo -e "\e[1;33m WARNING: You already have $PACKAGE installed with apt. \
Please uninstall it by running sudo apt-get purge $PACKAGE. The uav_ros_simulation \
would like to build it from source at \
https://github.com/larics/mav_comm/tree/larics_master. \e0"
  exit 2
fi

## | --------- change to the directory of this script --------- |

cd "$MY_PATH"

## | --------------------- install ROS ------------------------ |

bash $MY_PATH/dependencies/ros.sh

## | --------------------- install gitman --------------------- |

bash $MY_PATH/dependencies/gitman.sh

## | ---------------- install gitman submodules --------------- |

gitman install --force -v stack

# Install uav_ros_stack
bash $MY_PATH/../ros_packages/uav_ros_stack/installation/install.sh

gitman uninstall
gitman install --force -v $WHICH_STACK
gitman install --force -v simulation

# Install ardupilot
bash $MY_PATH/dependencies/ardupilot_dep.sh
bash $MY_PATH/../firmware/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y

SNAME=$( echo "$SHELL" | grep -Eo '[^/]+/?$' )
BASHRC=~/.$(echo $SNAME)rc

# Add Ardupilot exports to bashrc
num=`cat $BASHRC | grep "/ardupilot/Tools/autotest" | wc -l`
if [ "$num" -lt "1" ]; then

  TEMP=`( cd "$MY_PATH/../firmware/ardupilot/Tools/autotest" && pwd )`

  echo "Adding Ardupilot source to $BASHRC"
  echo "\
# Ardupilot exports
export PATH=\$PATH:$TEMP
export PATH=/usr/lib/ccache:\$PATH
" >> $BASHRC
fi

## | ------------- add Gazebo sourcing to .bashrc ------------- |

line="source /usr/share/gazebo/setup.sh"
num=`cat $BASHRC | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your $BASHRC"

  # set bashrc
  echo "$line" >> $BASHRC
fi

if [ -d "$MY_PATH/../firmware/ardupilot/Tools/completion" ]; then
  ## | ------------- add ardupilot completion ------------- |
  line=`( cd "$MY_PATH/../firmware/ardupilot/Tools/completion" && pwd )`
  num=`cat $BASHRC | grep "$line" | wc -l`
  if [ "$num" -lt "1" ]; then
    echo "Adding 'source $line/completion.$SNAME' to your $BASHRC"
    echo "source $line/completion.$SNAME" >> $BASHRC
  fi
fi

# Add mavproxy export in case we are using Ubuntu 20.04
line="export PATH=$HOME/.local/bin:\$PATH"
num=`cat $BASHRC | grep "$line" | wc -l`
if [[ "$distro" = "20.04" && "$num" -lt "1" ]]
then
  echo "Adding $line to $BASHRC"
  echo "$line" >> $BASHRC
fi

## | ------------- Build Ardupilot firmware ------------- |
export PATH="/usr/lib/ccache:$PATH"
export PATH="/opt/gcc-arm-none-eabi-6-2017-q2-update/bin:$PATH"
cd $MY_PATH/../firmware/ardupilot
modules/waf/waf-light configure --board sitl                    
modules/waf/waf-light build --target bin/arducopter

# Packages not needed for simulation
echo "Uninstalling enum34"
res=$(pip uninstall -y enum34)

echo "Installing empy"
res=$(pip install empy==3.3.4)

echo "Installing mavproxy==1.8.55"
res=$(pip install mavproxy==1.8.55)
