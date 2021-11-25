#!/bin/bash

#!/bin/bash

DRONEIP=128.238.39.130
source /opt/ros/melodic/setup.bash
source arpl_ws/devel/setup.bash    # Only work after catkin build
export ROS_MASTER_URI=http://$DRONEIP:11311

rosrun call_robot call_robot_srv.py --dummy