#!/bin/bash

cd /home/isa_afa_3_ws

source /opt/ros/jazzy/setup.bash

read -p "Do you wish to perform a clean installation? [y/n] " yn
case $yn in
    [Yy]* ) bash /home/isa_afa_3_ws/scripts/01_clear_workspace.bash;; # remove any files from a previous build
    [Nn]* ) ;;
    * ) ;;
esac

colcon build

source /home/isa_afa_3_ws/install/local_setup.bash
