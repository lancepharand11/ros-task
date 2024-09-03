#!/bin/bash

#source ~/.bashrc

#source /home/lancepharand11/robot_limo_ws/install/setup.bash
source /opt/ros/humble/setup.bash
source ~/robot_limo_ws/install/setup.bash

ros2 launch limo_simulation limo.launch.py

