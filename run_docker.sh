#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

echo "Running Docker Container"
CONTAINER_NAME=uav_ros_simulation

# Get distro of the built image
distro="focal-bin-0.0.1"
run_args=""
gpu_enabled=""

for (( i=1; i<=$#; i++));
do
  param="${!i}"

  if [ "$param" == "--enable-gpu" ]; then
    gpu_enabled="--gpus all"
  fi

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

run_args="$gpu_enabled $run_args"

echo "Running in $distro"

# Check if there is an already running container with the same distro
full_container_name="${CONTAINER_NAME}_${distro}_new"
running_container="$(docker container ls --all | grep $full_container_name)"
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
  --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env="XAUTHORITY=${XAUTH}" \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name $full_container_name \
  lmark1/uav_ros_simulation:$distro \
  /bin/bash
