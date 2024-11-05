# UAV ROS Simulation
A collection of ROS packages for Gazebo simulations of Ardupilot / PX4 UAV platforms

| UAV ROS Simulation build status | [![Melodic](https://github.com/larics/uav_ros_simulation/workflows/Melodic/badge.svg)](https://github.com/larics/uav_ros_simulation/actions)  | [![Noetic](https://github.com/larics/uav_ros_simulation/workflows/Noetic/badge.svg)](https://github.com/larics/uav_ros_simulation/actions) |
|-----------------------|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|

| ROS Package                                                                               | 18.04  | 20.04|
|-----------------------|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
| [ardupilot_gazebo](https://github.com/larics/ardupilot_gazebo) |  [![Melodic](https://github.com/larics/ardupilot_gazebo/workflows/Melodic/badge.svg)](https://github.com/larics/ardupilot_gazebo/actions) | [![Noetic](https://github.com/larics/ardupilot_gazebo/workflows/Noetic/badge.svg)](https://github.com/larics/ardupilot_gazebo/actions) |
| [ardupilot](https://github.com/larics/ardupilot) | [![Build SemaphoreCI](https://semaphoreci.com/api/v1/ardupilot/ardupilot/branches/master/badge.svg)](https://semaphoreci.com/ardupilot/ardupilot) | [![Build SemaphoreCI](https://semaphoreci.com/api/v1/ardupilot/ardupilot/branches/master/badge.svg)](https://semaphoreci.com/ardupilot/ardupilot) |
| [rotors_simulator](https://github.com/larics/rotors_simulator) | N/A | N/A |
| [mav_comm](https://github.com/larics/mav_comm) | N/A | N/A |
| [larics_gazebo_worlds](https://github.com/larics/larics_gazebo_worlds) | N/A | N/A |


## Installation

The uav_ros_simulation can be started only with docker. Find out how to work in a Docker environment at [HOWTO.md - Docker Commands](HOWTO.md). 
Docker can be installed with:
```bash
./installation/dependencies/docker.sh
```
Follow these steps for a quick and easy start.
1.  Pull the docker image. 
Prebuilt images for ```uav_ros_simulation``` found at [DockerHub lmark1/uav_ros_simulation](https://hub.docker.com/repository/docker/lmark1/uav_ros_simulation).
    ```bash 
    docker pull lmark1/uav_ros_simulation:focal-bin-0.2.1 
    ```
2. ```bash 
    ./docker_build.sh 
    ```
3. ```bash 
    ./docker_run.sh 
    ```

## Simulation Startup

UAV simulations are launched as tmuxinator sessions as follows.
```bash
cd ~/uav_ros_simulation/startup/kopterworx_one_flying
./start.sh
```

To find out more about navigating the simulation environment please read [HOWTO.md](HOWTO.md).

### Getting new changes from Github

If you want to update *uav_ros_simulation* and all its dependencies please run the following commands:
```bash
git pull
gitman update --skip-changes
catkin build
```

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

* [velodyne_simulator](https://github.com/larics/velodyne_simulator) - URDF description and Gazebo plugins to simulate Velodyne laser scanners
* [gazebo_ros_magnet](https://github.com/larics/storm_gazebo_ros_magnet) - This is a Gazebo model plugin that simulates a magnetic dipole-dipole model
* [larics_gazebo_worlds](https://github.com/larics/larics_gazebo_worlds) - This package contains various worlds used in Gazebo simulations
* [cartographer](https://github.com/larics/cartographer) - A package for real-time simultaneous localization and mapping (SLAM)
* [cartographer_ros](https://github.com/larics/cartographer_ros) - A package for SLAM that provides Cartographer's ROS integration
