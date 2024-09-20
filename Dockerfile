FROM ubuntu:22.04 AS slam-workshop-base
ENV DEBIAN_FRONTEND=noninteractive

# install basic packages
RUN apt-get update \
    && apt-get install curl -y \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list
RUN apt-get update \
    && apt-get install -y \
       python3-colcon-mixin \
       python3-pip \
       ros-dev-tools \
       ros-humble-ros-base \
       ros-humble-rqt* \
       ros-humble-rviz2

# install extra packages
COPY docker/packages.txt /root/
RUN apt-get update && xargs -a /root/packages.txt apt-get install -y
COPY docker/requirements.txt /root/
RUN pip install -r /root/requirements.txt

FROM slam-workshop-base AS slam-workshop-lint

RUN apt-get update \
    && apt-get install git -y
RUN pip3 install pysen black flake8 isort mypy types-PyYAML
# upgrade numpy because mypy is failed with default numpy version
RUN pip3 install -U numpy
