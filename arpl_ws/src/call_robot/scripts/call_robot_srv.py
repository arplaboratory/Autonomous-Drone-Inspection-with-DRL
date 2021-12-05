#!/usr/bin/python
# use system python2
# use the command line below before running this script
# rosrun call_robot call_robot_srv.py
# add --static to get image without sending the actual goto command
# rosservice call /call_robot "{x: 0.0, y: 0.0, z: -1.0, yaw: 0.0, filename: '/home/yang/image.png', topic: '/hires/image_raw/compressed', robot: 'dragonfly12'}"
import rospy
import argparse
import sys
sys.path.append('.')
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from call_robot.srv import CallRobot
from call_robot.srv import Vec4, Vec4Request

from utils import *
class call_robot_srv:
    def __init__(self,args):
        rospy.init_node('call_robot_server')
        self.bbox_gt = []
        self.bbox_pred = BoundingBoxes()
        self.img = []
        self.img_header = []
        self.odom = []
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_thresh = args.lt
        self.angular_thresh = args.at
        self.bbox = (Vec3(-0.45, -0.45, 0.0), Vec3(0.45, 0.45, 1.0))
        #self.bbox = (Vec3(-0.2, -0.2, 0.0), Vec3(0.2, 0.2, 0.5))
        self.s = rospy.Service('call_robot', CallRobot, self.handle_robot_call)
        rospy.Subscriber('/dragonfly12/odom', Odometry, self.odom_callback) 
        rospy.Subscriber('/hires/image_raw/compressed', CompressedImage, self.img_callback) 
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.pred_callback)
        rospy.Subscriber('/image_processor/output', BoundingBoxes, self.gt_callback)
    
    def handle_robot_call(self,req):
        #print("command: [%.2f, %.2f, %.2f, %.2f]\n filename: %s topic: %s robot: %s"%(req.x,req.y, req.z, req.yaw, req.filename, req.topic, req.robot))
        print("--------------------------\ncommand: [%.2f, %.2f, %.2f, %.2f]"%(req.x,req.y, req.z, req.yaw))
        # execute command
        failed = 0
        if not args.static:
            try:
                goto = rospy.ServiceProxy('/'+req.robot+'/'+'mav_services'+'/goTo', Vec4)
                post = Vec4Request()
                post.goal[0] = req.x
                post.goal[1] = req.y
                post.goal[2] = req.z
                post.goal[3] = req.yaw
                #print('before intersect detection')
                if intersect_line_segment_aabbox((Vec3(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z), Vec3(req.x, req.y, req.z)), self.bbox, debug=False):
                    print('intersect')
                    failed = 2
                else:
                    print('no intersect')
                    resp = goto(post)
                    #print('go_relative sucess')
                    if resp.success:
                        while not (self.linear_velocity < self.linear_thresh and self.angular_velocity < self.angular_thresh):
                            pass
                        #print('goto sucessed')
                    else:
                        #print('goto failed')
                        if not resp.success:
                            return "GoToFailed -1 -1 -1 -1 -1.0 -1 -1 -1 -1"
                        
                #print('after intersect detection')
            except Exception as e:
                print(e)
                failed=1
        if failed == 1:
            return "GoToException -1 -1 -1 -1 -1.0 -1 -1 -1 -1"
        elif failed == 2:
            return "Bump -1 -1 -1 -1 -1.0 -1 -1 -1 -1"
        # save image
        try:
            cv2.imwrite(req.filename, self.img)
            #print('gt header: ', self.bbox_gt)
            #print('pred header: ',self.bbox_pred)
            #print('image header ',self.img_header)
            output = [True, -1, -1, -1, -1, -1.0, -1, -1, -1, -1]
            num_bboxes = len(self.bbox_pred.bounding_boxes)
            if num_bboxes > 0:
                if ((self.bbox_pred.header.stamp == self.img_header.stamp)):
                    print('pred align')
                    tmp_max = -1.0
                    idx = 0
                    for i in range(num_bboxes):
                        if tmp_max < self.bbox_pred.bounding_boxes[i].probability:
                            tmp_max = self.bbox_pred.bounding_boxes[i].probability
                            idx = i
                    tmp = self.bbox_pred.bounding_boxes[idx]
                    output[1] = tmp.xmin
                    output[2] = tmp.ymin
                    output[3] = tmp.xmax
                    output[4] = tmp.ymax
                    output[5] = tmp.probability
            if len(self.bbox_gt.bounding_boxes) > 0:
                if ((self.bbox_gt.header.stamp == self.img_header.stamp)):
                    print('gt align')
                    tmp = self.bbox_gt.bounding_boxes[0]
                    output[6] = tmp.xmin
                    output[7] = tmp.ymin
                    output[8] = tmp.xmax
                    output[9] = tmp.ymax
            print('bbox: ', output)
            st = ''
            for item in output:
                st+= str(item)+' '
            return st
        except Exception as e:
            print(e)
            return "ImageException -1 -1 -1 -1 -1.0 -1 -1 -1 -1"
    def handle_dummy(req):
        print("command: [%.2f, %.2f, %.2f, %.2f]\n filename: %s\ntopic: %s\nrobot: %s"%(req.x,req.y, req.z, req.yaw, req.filename, req.topic, req.robot))
        x = np.zeros((256,265,3),np.uint8)
        try:
            cv2.imwrite(req.filename, x)
            return True
        except:
            return False

    def gt_callback(self,msg):
        self.bbox_gt = msg
    def pred_callback(self,msg):
        self.bbox_pred = msg
        #print('pred received')
    def odom_callback(self, msg):
        self.odom = msg
        self.linear_velocity = msg.twist.twist.linear.x * msg.twist.twist.linear.x + msg.twist.twist.linear.y * msg.twist.twist.linear.y + msg.twist.twist.linear.z + msg.twist.twist.linear.z
        self.angular_velocity = msg.twist.twist.angular.x * msg.twist.twist.angular.x + msg.twist.twist.angular.y * msg.twist.twist.angular.y + msg.twist.twist.angular.z * msg.twist.twist.angular.z
        #print(self.linear_velocity)
        #print(self.angular_velocity)
    def img_callback(self,msg):
        bridge = CvBridge()
        self.img_header = msg.header
        self.img = bridge.compressed_imgmsg_to_cv2(msg,"bgr8")
        #img = img[:, 80:-80]
        #self.img = cv2.resize(self.img, (256, 256), interpolation = cv2.INTER_AREA)


if __name__ == '__main__':
    # parse arguments
    CLI = argparse.ArgumentParser()
    CLI.add_argument("--static", action="store_true") 
    CLI.add_argument("--lt", type=float, default = 0.008) 
    CLI.add_argument("--at", type=float, default = 0.008) 
    args = CLI.parse_args()
    service = call_robot_srv(args)
    rospy.spin()
