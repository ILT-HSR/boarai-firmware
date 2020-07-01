#!/bin/bash

export ROS_DOMAIN_ID=1
source /home/hsr/code/firmware/install/setup.bash
ros2 launch boarai production.launch.py
