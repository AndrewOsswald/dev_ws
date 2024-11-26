#!/bin/bash
cd ..
cd ..
cd ..
cd /home/ubuntu/dev_ws

source install/local_setup.bash
source /opt/ros/foxy/setup.bash

cd launch

ros2 launch controller.launch.py
