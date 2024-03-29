#!/bin/bash

if [ -z "$1" ]; then
  echo "You must provide the path to the directory with the Dockerfile you want to build"
  echo "bash run-image.sh ./path/to/image"
  exit 1
fi

if [ ! -d "$1" ]; then
  echo "The path does not exist:"
  echo "$1"
  exit 1
fi

DOCKER_IMAGE=$1

# Run the developer's dockerfile
docker run -it --rm \
  --network=host \
  --privileged \
  --cpuset-cpus="0-5" \
  --memory=7000000000 \
  --volume=/Users/lorenzogentilini/Git/Others/CatkinWorkspace:/home/docker-dev/catkin_ws \
  --volume=/Users/lorenzogentilini/Git/Others/PX4-Autopilot:/home/docker-dev/PX4-Autopilot \
  --volume=/dev:/dev \
  $DOCKER_IMAGE
