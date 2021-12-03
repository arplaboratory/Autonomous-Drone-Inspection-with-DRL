#rostopic pub /dragonfly1/state_control/pos_cmd_flag quadrotor_msgs/AAUPosCmd "flag: true
#traj_name: 'random'" 

#echo "random traj"                          
#read -n 1 -s                                   

#rostopic pub /dragonfly1/state_control/pos_cmd_flag quadrotor_msgs/AAUPosCmd "flag: true
#traj_name: 'optimized'" 

#echo "optimized traj"                          

rosservice call /dragonfly1/mav_services/lissajous "{x_amp: 0.5, y_amp: 0.5, z_amp: 0.5, yaw_amp: 3.1415, x_num_periods: 15, y_num_periods: 10,
  z_num_periods: 10, yaw_num_periods: 0, period: 30, num_cycles: 1, ramp_time: 2}" 
echo "running lissajous"                          
read -n 1 -s                                   
