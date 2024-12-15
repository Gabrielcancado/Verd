#!/bin/bash

# Displays the current working directory to indicate the user is in the correct folder for the class.
echo "You have accessed the folder for the class: $PWD"

# Displays a message indicating that the container creation process is starting.
echo "Creating the container"

# Allows the Docker container to connect to the X server for graphical applications.
xhost +local:docker

# name ros_humble        -> Names the container as "ros_humble"
# user $(id -u):$(id -g) -> Sets the container user and group to match the host user and group
# -e DISPLAY=$DISPLAY    -> Passes the DISPLAY variable to allow the container to use the host's graphical interface
# -e QT_X11_NO_MITSHM=1  -> Fixes possible memory sharing issues with Qt and the X server
# -e XDG_RUNTIME_DIR=/tmp/runtime-ros2_ws -> Sets the temporary runtime directory for ROS
# --gpus all             -> Enables the use of all GPUs, if available
# --network=host         -> Configures the container to use the same network as the host
# --ipc=host             -> Shares IPC (inter-process communication) with the host
# --privileged           -> Grants the container elevated privileges to access devices and resources
# --device /dev/video0   -> Grants access to the video device (camera) in the container
# -v /dev/video0:/dev/video0 -> Maps the host's video device to the container
# -v /tmp/.X11-unix:/tmp/.X11-unix:rw -> Maps the X server socket to allow the container to use the graphical interface
# -v "$PWD/ros2_ws:/home/ros2_ws:rw" -> Maps the "ros2_ws" directory from the host to the container with read-write permissions
# --workdir /home        -> Sets the container's working directory to /home
# ros2_ws:humble         -> Specifies the image to use for the container, which is "ros2_ws:humble"

# Runs a Docker container with the specified options.
docker run -it --rm \
    $USE_GPUS \
    --name ros_humble \
    --user $(id -u):$(id -g) \
    -e HOME=/home/ros2_ws \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-ros2_ws \
    --network=host \
    --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$PWD/ros2_ws:/home/ros2_ws:rw" \
    --device /dev/dri:/dev/dri \
    --workdir /home/ros2_ws \
    ros2_ws:humble