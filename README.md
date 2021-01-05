# UAV ROS Simulation
A collection of ROS packages for Gazebo simulations of Ardupilot / PX4 UAV platforms

| UAV ROS Simulation build status | [![Melodic](https://github.com/lmark1/uav_ros_simulation/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_simulation/actions)  | [![Noetic](https://github.com/lmark1/uav_ros_simulation/workflows/Noetic/badge.svg)](https://github.com/lmark1/uav_ros_simulation/actions) |
|-----------------------|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|

| ROS Package                                                                               | 18.04  | 20.04|
|-----------------------|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
| [uav_ros_stack](https://github.com/lmark1/uav_ros_stack) | [![Melodic](https://github.com/lmark1/uav_ros_stack/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_stack/actions) | [![Noetic](https://github.com/lmark1/uav_ros_stack/workflows/Noetic/badge.svg)](https://github.com/lmark1/uav_ros_stack/actions) |
| [ardupilot_gazebo](https://github.com/larics/ardupilot_gazebo) | N/A | N/A |
| [rotors_simulator](https://github.com/larics/rotors_simulator) | N/A | N/A |
| [mav_comm](https://github.com/larics/mav_comm) | N/A | N/A |
| [ardupilot](https://github.com/larics/ardupilot) | [![Build Status](https://travis-ci.com/ArduPilot/ardupilot.svg?branch=master)](https://travis-ci.com/ArduPilot/ardupilot) | [![Build Status](https://travis-ci.com/ArduPilot/ardupilot.svg?branch=master)](https://travis-ci.com/ArduPilot/ardupilot) |

## Installation
### Advanced

Follow these installation steps if you have a catkin workspace already set up.

``` bash
# Navigate to src folder of the catkin workspace
cd /path/to/catkin_ws/src

# Install uav_ros_stack
git clone https://github.com/lmark1/uav_ros_simulation
cd uav_ros_simulation
./installation/install.sh
```