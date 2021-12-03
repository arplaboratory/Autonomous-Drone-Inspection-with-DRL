rosservice call /dragonfly2/mav_services/motors "data: true"
echo "dragonfly2 motors on"                          
read -n 1 -s                                   
rosservice call /dragonfly3/mav_services/motors "data: true"
echo "dragonfly3 motors on"                          
read -n 1 -s                                   
rosservice call /dragonfly6/mav_services/motors "data: true"
echo "dragonfly6 motors on"                          
