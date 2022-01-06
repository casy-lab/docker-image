# Docker Images

This repository contains Dockerfiles to build, and run, a docker image useful for executing the [UAV Software](https://github.com/casy-lab/uav-catkin-ws) on any quadcopter, regardless of the endowed companion computer.

### Dependencies

The built docker image already contains all the required dependencies, in particular:
  * ROS Noetic
  * Python 3
  * OpenCV 4
  * Ceres
  * Eigen
  * ACADO
  * NLopt

### Requirements

In order to use this repository you need to [install Docker](https://docs.docker.com/get-docker/) on your machine.
Make sure to follow the post-installation steps to [manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/), otherwise you will have to prepend `sudo` to the various commands.

### Using a Docker image

We provide convenience scripts for building Docker images and running containers from them.

```bash
./build-image.sh IMAGE_NAME
./run-image.sh IMAGE_NAME
```
