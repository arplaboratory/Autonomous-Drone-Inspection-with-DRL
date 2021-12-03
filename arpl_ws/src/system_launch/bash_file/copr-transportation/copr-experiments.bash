rosservice call /payload/mav_services/LoadgoTo "goal:
- 0.0
- 0.0
- 0.1
- 0.0" 

read -n 1 -s                                   
rosservice call /payload/mav_services/LoadgoTo "goal:
- 0.0
- 0.0
- 0.4
- 0.0" 
