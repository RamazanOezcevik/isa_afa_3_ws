FROM ros:jazzy

ENV DEBIAN_FRONTEND='noninteractive'
ENV TERM=xterm-256color
ENV HOME=/home

ARG ROS_DISTRO=jazzy
ARG GZ_DISTRO=harmonic

WORKDIR /home/isa_afa_3_ws

RUN apt-get update && apt upgrade -y
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    curl \
    lsb-release \
    gnupg \
    python3-pip \
    python3-rosdep \
    nano \
    screen \
    iputils-ping \
    iproute2 \
    ros-${ROS_DISTRO}-rviz2* \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-* \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-teleop-twist-keyboard

RUN yes | sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN yes | echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN yes | sudo apt-get update
RUN yes | sudo apt-get install gz-${GZ_DISTRO}
RUN yes | sudo apt-get install ros-${ROS_DISTRO}-ros-gz

RUN rm -rf /var/lib/apt/lists/*

RUN export ROS2_INSTALL_PATH=/opt/ros/jazzy
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\[\033[01;31m\]\u\[\033[01;34m\]@\[\033[01;36m\]\h\[\033[01;33m\]\w\[\033[01;35m\]\$ \[\033[00m\]'" >> ~/.bashrc
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
RUN echo 'source /home/isa_afa_3_ws/install/local_setup.bash' >> ~/.bashrc
RUN echo 'source /home/isa_afa_3_ws/scripts/set_gz_env.bash' >> ~/.bashrc

COPY "./.screenrc" "/home/.screenrc"
