FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Shanghai \
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
WORKDIR /workspace
RUN apt update && apt install -y software-properties-common
RUN yes | add-apt-repository universe
RUN apt update && \
    apt install -y curl git && \
    && ln -snf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
    && echo "Asia/Shanghai" > /etc/timezone

RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

RUN apt update && \
    apt install -y ros-humble-desktop ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl libyaml-cpp-dev python3-colcon-common-extensions
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
RUN source /opt/ros/humble/setup.bash
RUN git clone https://github.com/pndbotics/pnd_ros2 && \
    cd pnd_ros2/cyclonedds_ws/src && \
    git clone https://github.com/ros2/rmw_cyclonedds -b humble  && \
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x && \
    cd .. && \
    colcon build && \
    . install/setup.bash

