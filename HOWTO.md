# Simulation Startup Tutorial

## TMUX Keybinds

Running *start.sh* will automatically start and attach you to a tmux session, so it's useful to know tmux keybinds. 
The standard TMUX command prefix is **Ctrl+b**.

| Description |  Keybind |
|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
| Run tmux | :$ tmux |
| List tmux sessions | :$ tmux ls |
| Attach to a session | :$ tmux a -t [session name] |
| New Window | **prefix**+c |
| Move through windows (tabs) | Shift+&larr; &nbsp; Shift+&rarr; <br/> Alt+u &nbsp; Alt+i|
| Killing window | **prefix**+x |
| Killing session | **prefix**+k |
| Detaching from a session | **prefix**+d |
| Enter scroll mode | **prefix**+[ |
| Split pane horizontally |  **prefix**+% |
| Split pane vertically | **prefix**+" |

## ROS Topics and Services

Move UAVs by publishing on the following topic:
```bash
rostopic pub /red/tracker/input_pose   # Namespace prefix either red, blue or yellow
```

Additional useful MAVProxy commands can be found at [ardupilot_gazebo/README.md/Simulation and Mavproxy Commands](https://github.com/larics/ardupilot_gazebo/blob/larics-master/README.md#simulation-and-mavproxy-commands).

## Docker Commands

Most information found at the [Docker Curriculum](https://docker-curriculum.com/).

### Introduction

| Description   |      Command      |
|:---------|:------------------------:|
| Run an example docker file |  ```docker run hello-world``` |
| List all the containers (running and exited) |    ```docker ps -a```   |

### Running Containers

| Description   |      Command      |
|:---------|:------------------------:|
| Run a command in a docker container.<br/> ```-p``` publish containers ports to the host e.g. port 80 -> 8888 <br/> ```-d``` run conatiner in background <br/> ```--rm``` automatically delete the container once it exits <br/> ```-P``` publish all exposed ports <br/> ```--name``` name your container for easier use in subsequent commands <br/> ```-e``` set environment variables | ```docker run -p 8888:80 -d ubuntu:latest``` |
| See all the ports that container is running | ```docker port [CONTAINER] ```|
| Run a ```/bin/bash``` command in the ubuntu container <br/> ```-i``` connect the open I/O process streams  <br/> ```-t``` with an allocated TTY terminal |```docker run -it --name mydocker ubuntu /bin/bash```|

### Managing Containers

| Description   |      Command      |
|:---------|:------------------------:|
| Stop the docker image (go to EXIT state) | ```docker stop mydocker``` |
| Kill a container from a running state | ```docker kill mydocker``` | 
| Remove a conatiner | ```docker rm mydocker``` |
| Remove all containers | ```docker rm $(docker ps -a -q)```|
| Remove all stopped containers | ```docker container prune```|
| List all docker containers | ```docker container ls``` |
| Get all the logs| ```docker container logs [CONTAINER_NAME]```|
| Remove everything Docker related | ``` docker system prune --all --volumes```|
| Start an already created (a.k.a. "docker run"-ed) container | ``` docker start [CONTAINER_NAME]```|
| Attach to an already started container | ```docker attach [CONTAINER_NAME]```|
| Detach while inside a container | ```Ctrl+P Ctrl+Q```|

### Working with DockerHub registry

| Description   |      Command      |
|:---------|:------------------------:|
|Fetch an image from the DockerHub registry | ```docker pull busybox```|
|Push an image to the DockerHub registry | ```docker push [IMAGE_NAME]```|
| Search the DockerHub registry for images | ```docker search [IMAGE_NAME]``` |

### Building Docker Images

| Description   |      Command      |
|:---------|:------------------------:|
| Build a Docker image using a ```Dockerfile``` found in the current directory <br/> ```-t``` a tag name | ```docker build -t myusername/hello_docker .```|

### Docker Network

| Description   |      Command      |
|:---------|:------------------------:|
| Create a new ```bridge``` Docker network <br/> Allows containers connected to the same bridge to communicate | ```docker network create my-network``` |
| List all docker networks | ```docker network ls``` |

### Run GUI applications

To run GUI application on the host append ```-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix``` to the ```docker run``` command.  
*NOTE* Before running the docker command make sure to disable access control on the host machine by running ```xhost +```. To enable access control run ```xhost -```.
### docker-compose

| Description   |      Command      |
|:---------|:------------------------:|
| Run ```docker-compose.yml``` in the current directory <br/> ```-d``` run in detached mode | ```docker-compose up -d``` |
| Teardown the active compose <br/> ```-v``` delete volumes | ```docker-compose down -v``` |

