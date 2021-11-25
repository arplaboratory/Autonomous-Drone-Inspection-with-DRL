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

### Send relative position command and write image
- In the first window
```bash
DRONEIP=128.238.39.130
export ROS_MASTER_URI=http://$DRONEIP:11311
rosrun call_robot call_robot_srv.py
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
DRONE_IP=128.238.39.130
export ROS_MASTER_URI=http://$DRONEIP:11311
ROBOT=dragonfly12
alias mon="rosservice call /$ROBOT/mav_services/motors true"
alias moff="rosservice call /$ROBOT/mav_services/motors false"
alias takeoff="rosservice call /$ROBOT/mav_services/takeoff"
alias land="rosservice call /$ROBOT/mav_services/land"
alias lande="rosservice call /$ROBOT/mav_services/goTo \"goal: [0.0, 0.0, 0.2, 0.0]\""
# mon: turn on motors
# moff: turn off motors
# takeoff: takeoff the drone
# land: land the drone, then moff
# lande: land the drone when land is not responding, then moff
# rosservice call can only work after the takeoff
``` 

- In the second window
```bash
USER=jiuhong
DRONEIP=128.238.39.130
export ROS_MASTER_URI=http://$DRONEIP:11311
rosservice call /call_robot "{x: 0.0, y: 0.0, z: -1.0, yaw: 0.0, filename: '/home/$USER/image.png', topic: '/hires/image_raw/compressed', robot: 'dragonfly12'}"
```

### Dummy mode
- In the first window
```bash
DRONEIP=128.238.39.127
export ROS_MASTER_URI=http://$DRONEIP:11311
rosrun call_robot call_robot_srv.py --dummy
```
- In the second window
```bash
USER=jiuhong
DRONEIP=128.238.39.127
export ROS_MASTER_URI=http://$DRONEIP:11311
rosservice call /call_robot "{x: 0.0, y: 0.0, z: -1.0, yaw: 0.0, filename: '/home/$USER/image.png', topic: '/hires/image_raw/compressed', robot: 'dragonfly12'}"
```
