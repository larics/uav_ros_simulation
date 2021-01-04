#!/bin/bash

# Exit immediatelly if a command exits with a non-zero status
set -e

# Executes a command when DEBUG signal is emitted in this script - should be after every line
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# Executes a command when ERR signal is emmitted in this script
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

echo "$0: installing Gitman"

distro=`lsb_release -r | awk '{ print $2 }'`

if [ "$distro" = "18.04" ]; then
  sudo apt-get -y install python-pip python3-pip python-setuptools python3-setuptools
elif [ "$distro" = "20.04" ]; then
  sudo apt-get -y install python3-pip python3-setuptools
fi

sudo pip3 install gitman
sudo -H pip3 install gitman