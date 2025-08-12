#!/bin/bash

echo 'Registering source for rosdep and updating apt'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt-get update

cd /home/isa_afa_3_ws

echo 'Updating rosdeps'
rosdep update --rosdistro jazzy
rosdep install --from-path src --ignore-src -r -y

echo 'Building workspace'
source /opt/ros/jazzy/setup.bash

source /home/isa_afa_3_ws/install/local_setup.bash
