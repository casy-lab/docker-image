# ********************************************************
# Ubuntu OS, User and Base Environment
# ********************************************************
FROM ubuntu:20.04 AS ubuntu-base

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive

# Make apt to always ignore recommended and suggested packages
# This is particularly important with rosdep which invoked apt without `--no-install-recommends`
RUN echo \
  'APT::Install-Recommends "0";\nAPT::Install-Suggests "0";' > /etc/apt/apt.conf.d/01norecommend

# Install sudo
RUN apt-get update && apt-get install -q -y --no-install-recommends sudo && \
  rm -rf /var/lib/apt/lists/*

# Build-time arguments
ARG USERNAME=docker-dev
ARG GROUPNAME=$USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set username and home environment variables
ENV USERNAME $USERNAME
ENV HOME /home/$USERNAME

# Create a new user with the provided details
RUN groupadd --gid $USER_GID $GROUPNAME && \
  useradd --create-home --home-dir /home/$USERNAME --shell /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME

# Add the new user to the sudoers group with no password
RUN echo "$USERNAME:x:$USER_UID:$USER_GID:Developer,,,:$HOME:/bin/bash" >> /etc/passwd && \
  echo "$USERNAME:x:$USER_GID:" >> /etc/group && \
  echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME && \
  chown $USER_UID:$USER_GID -R $HOME

# Install and setup fixuid
RUN apt-get update && apt-get install -q -y --no-install-recommends curl ca-certificates && \
  curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.5.1/fixuid-0.5.1-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
  chown root:root /usr/local/bin/fixuid && \
  chmod 4755 /usr/local/bin/fixuid && \
  mkdir -p /etc/fixuid && \
  printf "user: $USERNAME\ngroup: $GROUPNAME\n" > /etc/fixuid/config.yml && \
  rm -rf /var/lib/apt/lists/*

# Setup locale and timezone
RUN echo "Europe/Rome" | sudo tee /etc/timezone > /dev/null && \
  sudo ln -fs /usr/share/zoneinfo/Europe/Rome /etc/localtime && \
  sudo apt-get update && sudo apt-get install -q -y --no-install-recommends locales tzdata && \
  sudo rm -rf /var/lib/apt/lists/*
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# ********************************************************
# Ubuntu Development Tools
# ********************************************************
FROM ubuntu-base AS ubuntu-dev

# C++ development
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    build-essential \
    cmake \
    gcc \
    g++ \
  && sudo rm -rf /var/lib/apt/lists/*

# Python development
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    python3 \
    python3-dev \
    python3-pip \
  && sudo rm -rf /var/lib/apt/lists/*
RUN pip3 install --upgrade pip

# Install additional dependencies
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    software-properties-common \
    git \
    nano \
    vim \
  && sudo rm -rf /var/lib/apt/lists/*

# ********************************************************
# Uav Base Dependencies
# ********************************************************
FROM ubuntu-dev AS uav-base

## Install OpenCV ##########
# Install OpenCV Dependencies
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    checkinstall \
    pkg-config \
    yasm \
    gfortran \
    libjpeg8-dev \
    libpng-dev \
    libtiff5-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libdc1394-22-dev \
    libxine2-dev \
    libv4l-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    qt5-default \
    libgtk2.0-dev \
    libtbb-dev \
    libatlas-base-dev \
    libfaac-dev \
    libmp3lame-dev \
    libtheora-dev \
    libvorbis-dev \
    libxvidcore-dev \
    libopencore-amrnb-dev \
    libopencore-amrwb-dev \
    x264 \
    v4l-utils \
  && sudo rm -rf /var/lib/apt/lists/*

# Optional OpenCV Dependencies
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    libprotobuf-dev \
    protobuf-compiler \
    libgoogle-glog-dev \
    libgflags-dev \
    libgphoto2-dev \
    libeigen3-dev \
    libhdf5-dev \
    doxygen \
  && sudo rm -rf /var/lib/apt/lists/*

# Python Dependencies
RUN pip3 install --upgrade numpy scipy matplotlib scikit-image scikit-learn ipython

# Build OpenCV From Source
RUN mkdir -p /workspace/opencv
WORKDIR /workspace/opencv
RUN git clone https://github.com/opencv/opencv.git opencv
RUN git clone https://github.com/opencv/opencv_contrib.git opencv_contrib

WORKDIR /workspace/opencv/opencv_contrib
RUN git checkout tags/4.5.5

WORKDIR /workspace/opencv/opencv
RUN git checkout tags/4.5.5

RUN mkdir -p /workspace/opencv/opencv/build
WORKDIR /workspace/opencv/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D INSTALL_C_EXAMPLES=ON \
          -D INSTALL_PYTHON_EXAMPLES=ON \
          -D WITH_TBB=ON \
          -D WITH_V4L=ON \
          -D WITH_QT=ON \
          -D WITH_OPENGL=ON \
          -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
          -D BUILD_EXAMPLES=ON ..

RUN make
RUN make install
############################

## Install Ceres Solver ##########
# Install Ceres Dependencies
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    libsuitesparse-dev \
  && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
RUN git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver

RUN mkdir -p /workspace/ceres-solver/build
WORKDIR /workspace/ceres-solver/build
RUN cmake ..
RUN make
RUN make install
############################

## Install Ceres Solver ##########
WORKDIR /workspace
RUN git clone https://github.com/stevengj/nlopt.git nlopt

WORKDIR /workspace/nlopt
RUN git checkout tags/v2.6.2

RUN mkdir -p /workspace/nlopt/build
WORKDIR /workspace/nlopt/build
RUN cmake ..
RUN make
RUN sudo make install
############################

## Install ACADO Toolkit ##########
# Install ACADO Dependencies
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    gnuplot \
    graphviz \
  && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
RUN git clone https://github.com/acado/acado.git -b stable ACADOtoolkit

RUN mkdir -p /workspace/ACADOtoolkit/build
WORKDIR /workspace/ACADOtoolkit/build
RUN cmake ..
RUN make
############################

## Install ROS Noetic ##########
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    gnupg \ 
    lsb-release \
    curl \ 
  && sudo rm -rf /var/lib/apt/lists/*

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Specify which ros version we use (needed by packages that support multiple ones)
ENV ROS_DISTRO noetic

RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    ros-$ROS_DISTRO-desktop-full \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-extras \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-octomap \
    ros-$ROS_DISTRO-octomap-ros \
    ros-$ROS_DISTRO-octomap-msgs \
    ros-$ROS_DISTRO-dynamic-edt-3d \
  && sudo rm -rf /var/lib/apt/lists/*
############################

## Install LibRealSense ##########
# Install LibRealSense Dependencies
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libusb-1.0-0-dev \
  && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
RUN git clone https://github.com/IntelRealSense/librealsense librealsense

RUN mkdir -p /workspace/librealsense/build
WORKDIR /workspace/librealsense/build
RUN cmake .. -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release
RUN make
RUN sudo make install

# Symlink udev rules
RUN sudo ln -s /workspace/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules
############################

# Additional Dependencies
RUN sudo add-apt-repository ppa:alex-p/tesseract-ocr5
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    python3-pulp \
    tesseract-ocr \
  && sudo rm -rf /var/lib/apt/lists/*

# Additional Python Dependencies
RUN pip3 install --upgrade bezier rtree

# Start using the new user
USER $USERNAME:$GROUPNAME

# Set Up Bashrc
RUN echo '\n\
  source /opt/ros/noetic/setup.bash' >> $HOME/.bashrc

# Switch to user's HOME folder
WORKDIR $HOME

ENTRYPOINT ["fixuid", "-q"]
CMD ["bash"]
