#!/usr/bin/python
# use system python2
# use the command line below before running this script
# export ROS_MASTER_URI=http://128.238.39.130:11311
# rosrun call_robot call_robot_srv.py
# rosrun call_robot call_robot_srv.py --dummy
# add --dummy to get black 256*256*3 image
# rosservice call /call_robot "{x: 0.0, y: 0.0, z: -1.0, yaw: 0.0, filename: '/home/yang/image.png', topic: '/hires/image_raw/compressed', robot: 'dragonfly12'}"
import rospy
import argparse
import sys
sys.path.append('.')
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from call_robot.srv import CallRobot
from call_robot.srv import Vec4, Vec4Request
def handle_robot_call(req):
    print("command: [%.2f, %.2f, %.2f, %.2f]\n filename: %s\ntopic: %s\nrobot: %s"%(req.x,req.y, req.z, req.yaw, req.filename, req.topic, req.robot))
    # execute command
    try:
        goto_relative = rospy.ServiceProxy('/'+req.robot+'/'+'mav_services'+'/goToRelative', Vec4)
        post = Vec4Request()
        post.goal[0] = req.x
        post.goal[1] = req.y
        post.goal[2] = req.z
        post.goal[3] = req.yaw
        resp = goto_relative(post)
        #print('go_relative sucess')
        print(resp.success)
    except:
        return False
    # save image
    try:
        bridge = CvBridge()
        msg = rospy.wait_for_message(req.topic, CompressedImage)
        img  = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        img  = img[:,80:-80]
        img  = cv2.resize(img, (256,256), interpolation = cv2.INTER_AREA)
        cv2.imwrite(req.filename, img)
        return True
    except:
        return False
def handle_dummy(req):
    print("command: [%.2f, %.2f, %.2f, %.2f]\n filename: %s\ntopic: %s\nrobot: %s"%(req.x,req.y, req.z, req.yaw, req.filename, req.topic, req.robot))
    x = np.zeros((256,265,3),np.uint8)
    try:
        cv2.imwrite(req.filename, x)
        return True
    except:
        return False
if __name__ == '__main__':
    # parse arguments
    CLI = argparse.ArgumentParser()
    CLI.add_argument("--dummy", action="store_true") 
    args = CLI.parse_args()
    # dummy execution return black image
    rospy.init_node('call_robot_server')
    if args.dummy:
        print('dummy')
        s = rospy.Service('call_robot', CallRobot, handle_dummy)
    else:
        print('normal')
        s = rospy.Service('call_robot', CallRobot, handle_robot_call)
    rospy.spin()
