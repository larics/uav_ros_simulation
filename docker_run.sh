#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

echo "Running Docker Container"
CONTAINER_NAME=uav_ros_simulation

# Get distro of the built image
distro=$(docker images $CONTAINER_NAME | tail -n1 | awk '{print $2}')
run_args=""

for (( i=1; i<=$#; i++));
do
  param="${!i}"

  if [ "$param" == "--bionic" ]; then
    distro="bionic"
  fi

  if [ "$param" == "--focal" ]; then
    distro="focal"
  fi

  if [ "$param" == "--focal-nogpu" ]; then
    distro="focal-nogpu"
  fi

  if [ "$param" == "--run-args" ]; then
    j=$((i+1))
    run_args="${!j}"
  fi

done

# Check if using GPU
gpu_enabled="--gpus all"
if [ "$distro" == "focal-nogpu" ]; then
  gpu_enabled=""
fi
run_args="$gpu_enabled $run_args"

echo "Running in $distro"

# Check if there is an already running container with the same distro
full_container_name="${CONTAINER_NAME}_${distro}"
running_container="$(docker container ls -al | grep $full_container_name)"
if [ -z "$running_container" ]; then
  echo "Running $full_container_name for the first time!"
else
  echo "Found an open $full_container_name container. Starting and attaching!"
  eval "docker start $full_container_name"
  eval "docker attach $full_container_name"
  exit 0
fi

# Hook to the current SSH_AUTH_LOCK - since it changes
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock

set -x
docker run \
  $run_args \
  -it \
  --network host \
  --privileged \
  --volume /dev:/dev \
  --volume /tmp/.x11-unix:/tmp/.x11-unix \
  --volume $HOME/bags:/root/bags \
  --volume $HOME/rosbag_default:/root/rosbag_default \
  --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --env="XAUTHORITY=${XAUTH}" \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name $full_container_name \
  uav_ros_simulation:focal \
  /bin/bash