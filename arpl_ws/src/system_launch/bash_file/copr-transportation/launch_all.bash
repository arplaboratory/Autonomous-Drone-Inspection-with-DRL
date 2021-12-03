rosservice call /payload/mav_services/Loadtakeoff 
echo "robots taking off"                          
read -n 1 -s                                   

rosservice call /payload/mav_services/Loadland 
echo "robots landing off"                          
read -n 1 -s                                   

rosservice call /dragonfly2/mav_services/motors "data: false"
echo "dragonfly2 motors off"                          
#read -n 1 -s                                   
rosservice call /dragonfly3/mav_services/motors "data: false"
echo "dragonfly3 motors off"                          
#read -n 1 -s                                   
rosservice call /dragonfly6/mav_services/motors "data: false"
echo "dragonfly6 motors off"                          
