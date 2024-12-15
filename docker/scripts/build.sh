#!/bin/bash

# Changes the current directory to the "docker" directory.
cd docker

# Displays a message confirming the user is in the "docker" folder, 
# and shows the current working directory.
echo "You have accessed the folder for the class: $PWD"

# Displays a message indicating that the ROS image is being built.
echo "Building the ROS image"

# Builds a Docker image using the specified Dockerfile.
docker build \
    --network=host \
    -f ROS_humble.dockerfile \
    -t ros2_ws:humble \
    --rm \
    .

# Exits the script with a success code (0).
exit 0
