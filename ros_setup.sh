#!/bin/bash

source /opt/ros/melodic/setup.bash
cd arpl_ws
catkin build
echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
echo 'source /home/jiuhong/Autonomous-Drone-Inspection-with-DRL/arpl_ws/devel/setup.bashrc' >> ~/.bashrc
