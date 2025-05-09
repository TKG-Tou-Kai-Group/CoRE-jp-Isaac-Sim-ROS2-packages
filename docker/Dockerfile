FROM nvcr.io/nvidia/isaac-sim:4.2.0

ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  apt-get install -y -qq \
    libyaml-cpp-dev curl software-properties-common \
    vim v4l-utils exfat-* \
    openssh-server \
    terminator dbus-x11 \
    python3-pip && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt update && \
    apt install -y -qq cmake

# for ros2
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
RUN apt -y -qq update && apt -y -qq install software-properties-common mesa-utils libglu1-mesa-dev\
  freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev \
  libao-dev libmpg123-dev libasio-dev && \
  add-apt-repository universe && \
  apt -y -qq update && apt install curl -y && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  apt -y -qq update && \
  apt -y -qq upgrade && \
  apt -y -qq install ros-humble-desktop \
    ros-humble-ament-cmake ros-humble-angles ros-humble-controller-manager \
    ros-humble-pluginlib ros-humble-urdf ros-humble-yaml-cpp-vendor ros-humble-joint-state-pub* \
    ros-humble-cv-bridge ros-humble-diagnostic-updater && \
  apt-get install -y -qq ros-humble-xacro ros-humble-realtime-tools ros-humble-control-toolbox ros-humble-ros2-control* \
    ros-humble-teleop-twist-keyboard ros-humble-vision-msgs ros-humble-rosbridge-suite && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*
  
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    telnet \
    nodejs \
    && apt-get clean

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp ros-humble-ros-testing \
    && apt-get clean

#RUN echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/root/colcon_ws/src/isaac_ros2_utils/isaac_ros2_scripts/config/fastdds.xml' >> ~/.bashrc
#RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /root/colcon_ws/install/setup.sh' >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
