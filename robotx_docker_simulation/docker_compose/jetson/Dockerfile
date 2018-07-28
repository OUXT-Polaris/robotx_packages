FROM nvidia/cuda:9.0-cudnn7-runtime

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=kinetic \
    DEBIAN_FRONTEND=noninteractive \
    USER_NAME=ubuntu

RUN apt-get update && apt-get install -q -y \
    apt-utils \
    curl \
    dirmngr \
    gnupg2 \
    less \
    lsb-release \
    lxde-core \
    && rm -rf /var/lib/apt/lists/*

# install ros-kinetic
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-ros-base\
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init
RUN useradd -m $USER_NAME && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/${USER_NAME}

USER $USER_NAME
RUN rosdep update

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER_NAME/.bashrc
RUN mkdir -p /home/$USER_NAME/catkin_ws/src
WORKDIR /home/$USER_NAME/catkin_ws/src
RUN git clone https://github.com/OUXT-Polaris/robotx_packages.git
WORKDIR /home/$USER_NAME/catkin_ws/src/robotx_packages/
RUN rm -rf robotx_tools
RUN rm -rf robotx_gazebo
RUN rm -rf robotx_docker_simulation
WORKDIR /home/$USER_NAME/catkin_ws/

RUN sudo apt-get update && \
    export DEBIAN_FRONTEND=noninteractive && \
    rosdep install -i -r -y --from-paths src --rosdistro kinetic

RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && \
    rm -rf devel build && \
    catkin_make_isolated"
RUN echo "source /home/$USER_NAME/catkin_ws/devel_isolated/setup.bash" >> /home/$USER_NAME/.bashrc