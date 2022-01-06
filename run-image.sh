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
CMD=${@:2}

# Run the developer's dockerfile
docker run -it --rm \
  --network=host \
  --privileged \
  --volume=/Users/lorenzogentilini/Git/Others/CatkinWorkspace:/home/docker-dev/catkin_ws \
  --volume=/dev:/dev \
  $DOCKER_IMAGE \
  $CMD