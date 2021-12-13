#!/bin/bash

# Enable access control for GUI 
xhost +

docker run \
  --gpus all \
  -it \
  --rm \
  --network host \
  --privileged \
  --volume /dev:/dev \
  --volume /tmp/.x11-unix:/tmp/.x11-unix \
  --env DISPLAY=$DISPLAY \
  --name uav_ros_simulation \
  lmark1/uav_ros_simulation:focal \
  /bin/bash

# Disable access control for GUI
xhost -
