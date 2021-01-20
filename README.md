# UAV ROS Simulation
A collection of ROS packages for Gazebo simulations of Ardupilot / PX4 UAV platforms

| UAV ROS Simulation build status | [![Melodic](https://github.com/lmark1/uav_ros_simulation/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_simulation/actions)  | [![Noetic](https://github.com/lmark1/uav_ros_simulation/workflows/Noetic/badge.svg)](https://github.com/lmark1/uav_ros_simulation/actions) |
|-----------------------|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|

| ROS Package                                                                               | 18.04  | 20.04|
|-----------------------|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
| [uav_ros_stack](https://github.com/lmark1/uav_ros_stack) | [![Melodic](https://github.com/lmark1/uav_ros_stack/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_stack/actions) | [![Noetic](https://github.com/lmark1/uav_ros_stack/workflows/Noetic/badge.svg)](https://github.com/lmark1/uav_ros_stack/actions) |
| [ardupilot_gazebo](https://github.com/larics/ardupilot_gazebo) |  [![Melodic](https://github.com/larics/ardupilot_gazebo/workflows/Melodic/badge.svg)](https://github.com/larics/ardupilot_gazebo/actions) | [![Noetic](https://github.com/larics/ardupilot_gazebo/workflows/Noetic/badge.svg)](https://github.com/larics/ardupilot_gazebo/actions) |
| [ardupilot](https://github.com/larics/ardupilot) | [![Build SemaphoreCI](https://semaphoreci.com/api/v1/ardupilot/ardupilot/branches/master/badge.svg)](https://semaphoreci.com/ardupilot/ardupilot) | [![Build SemaphoreCI](https://semaphoreci.com/api/v1/ardupilot/ardupilot/branches/master/badge.svg)](https://semaphoreci.com/ardupilot/ardupilot) |
| [rotors_simulator](https://github.com/larics/rotors_simulator) | N/A | N/A |
| [mav_comm](https://github.com/larics/mav_comm) | N/A | N/A |


## Installation

### Beginner

Follow these steps for a quick and easy start.

```bash
git clone https://github.com/lmark1/uav_ros_simulation
cd uav_ros_simulation/installation
./install_and_setup_workspace.sh
catkin build
source ~/.bashrc
```

### Advanced

Follow these installation steps if you have a catkin workspace already set up.

``` bash
# Navigate to src folder of the catkin workspace
cd /path/to/catkin_ws/src

# Install uav_ros_stack
git clone https://github.com/lmark1/uav_ros_simulation
cd uav_ros_simulation
./installation/install.sh

# Build catkin workspace
catkin build

# Export GAZEBO_PLUGIN_PATH
./installation/gazebo/setup_gazebo.sh /path/to/catkin_ws/build

# Source ~/.basrhc or ~/.zshrc
source ~/.bashrc
```

## Startup

UAV simulations are launched as tmuxinator sessions as follows.
```bash
cd uav_ros_simulation/startup/kopterworx_flying
./start.sh
```

To find out more about navigating the simulation environment please read [HOWTO.md](HOWTO.md).

## Troubleshooting

### ~/.bashrc

Make sure the following lines or their equivalents are found at the end of your ~/.bashrc (or ~/.zshrc) file in any order 
after installataion:
```bash
source /opt/ros/noetic/setup.bash
source /root/uav_ws/devel/setup.bash
source /usr/share/gazebo/setup.sh

# shell configuration
source /uav_ros_simulation/ros_packages/uav_ros_stack/miscellaneous/shell_additions/shell_scripts.sh

# Ardupilot exports
export PATH=$PATH:/root/uav_ws/src/uav_ros_simulation/firmware/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH

# ardupilot_gazebo exports
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/root/uav_ws/build/ardupilot_gazebo
```
**NOTE**: Lines should be automatically added. If some lines are missing please report an issue and add those lines manually.

## Optional Dependencies

Depending on the simulation type additional ROS packages may need to be installed.

* [velodyne_simulator](https://github.com/lmark1/velodyne_simulator) - URDF description and Gazebo plugins to simulate Velodyne laser scanners
* [gazebo_ros_magnet](https://github.com/larics/storm_gazebo_ros_magnet) - This is a Gazebo model plugin that simulates a magnetic dipole-dipole model
* [larics_gazebo_worlds](https://github.com/larics/larics_gazebo_worlds) - This package contains various worlds used in Gazebo simulations