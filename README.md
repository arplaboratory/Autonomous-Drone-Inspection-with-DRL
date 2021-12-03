# Autonomous-Drone-Inspection-with-DRL
## Robot Platform
### Environment Setup
- If you use bash
```bash
source /opt/ros/melodic/setup.bash
cd arpl_ws
catkin build
echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
echo 'source <PATH_TO_ARPL_WS>/arpl_ws/devel/setup.bashrc' >> ~/.bashrc
```
- If you use zsh
```bash
source /opt/ros/melodic/setup.zsh
cd arpl_ws
catkin build
echo 'source /opt/ros/melodic/setup.zsh' >> ~/.zshrc
echo 'source <PATH_TO_ARPL_WS>/arpl_ws/devel/setup.zshrc' >> ~/.zshrc
```

- Build ROS1 workspace
```bash
cd arpl_ws
catkin build
```

- Fly drone
```
# 1.ssh to drone, passwd: linaro
ssh dragonfly12@dragonfly12
sudo -s
# 2. In the first window
roslaunch mav_launch state_control.launch
# or use 'st' for alias
# 3. In the second window
roslaunch snap_cam_ros hires.launch
# 4. Back to user PC bash
#   a) If you do not want to use multimaster to sync ros1 master
DRONE_IP=128.238.39.130
export ROS_MASTER_URI=http://$DRONEIP:11311
#   b) If you want to use multimaster
roslaunch fkie_master_discovery master_discovery.launch
roslaunch fkie_master_sync master_sync_drl.launch
#   c) setup bashrc alias for easy drone control
export ROBOT=dragonfly12
alias mon="rosservice call /$ROBOT/mav_services/motors true"
alias moff="rosservice call /$ROBOT/mav_services/motors false"
alias takeoff="rosservice call /$ROBOT/mav_services/takeoff"
alias land="rosservice call /$ROBOT/mav_services/land"
alias lande="rosservice call /$ROBOT/mav_services/goTo \"goal: [0.0, 0.0, 0.2, 0.0]\""
alias home="rosservice call /$ROBOT/mav_services/goHome"
# mon: turn on motors
# moff: turn off motors
# takeoff: takeoff the drone
# land: land the drone, then moff
# lande: land the drone when land is not responding, then moff
# home: go the initial position
# rosservice call can only work after the takeoff
``` 

- One-stop launch file for fkie_master_discovery, fkie_master_sync, darknet_ros, py_image_processor, mocap_vicon
```bash
roslaunch system_launch api_system_launch.launch
```

- launch the call_robot services
```bash
rosrun call_robot call_robot_srv.py --lt 0.008 --at 0.008
# lt: linear velocity threshold for stopping
# at: augular velocity threshold for stopping
```

- How to use the call_robot
```bash
rosservice call /call_robot "{x: 0.0, y: 0.0, z: -1.0, yaw: 0.0, filename: '/home/$USER/image.png', topic: '/hires/image_raw/compressed', robot: 'dragonfly12'}"
```
