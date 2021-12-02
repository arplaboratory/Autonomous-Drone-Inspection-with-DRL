#!/bin/bash

source /opt/ros/melodic/setup.bash
cd arpl_ws
catkin build
echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
echo 'source <PATH_TO_ARPL_WS>/arpl_ws/devel/setup.bashrc' >> ~/.bashrc
