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
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from call_robot.srv import CallRobot
from call_robot.srv import Vec4, Vec4Request
    
class call_robot_srv:
    def __init__(self,args):
        rospy.init_node('call_robot_server')
        if args.dummy:
            print('dummy')
            s = rospy.Service('call_robot', CallRobot, handle_dummy)
        else:
            print('normal')
            self.bbox_gt = []
            self.bbox_pred = []
            self.img = []
            self.img_header = []
            self.s = rospy.Service('call_robot', CallRobot, self.handle_robot_call)
            rospy.Subscriber('/hires/image_raw/compressed', CompressedImage, self.img_callback) 
            rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.gt_callback)
            rospy.Subscriber('/image_processor/output', BoundingBoxes, self.pred_callback)
    
    def handle_robot_call(self,req):
        print("command: [%.2f, %.2f, %.2f, %.2f]\n filename: %s\ntopic: %s\nrobot: %s"%(req.x,req.y, req.z, req.yaw, req.filename, req.topic, req.robot))
        # execute command
        if not args.static:
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
            #bridge = CvBridge()
            #msg = rospy.wait_for_message(req.topic, CompressedImage)
            #img  = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            #img  = img[:,80:-80]
            #img  = cv2.resize(img, (256,256), interpolation = cv2.INTER_AREA)
            #msg_bbox_gt = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes)
            #msg_bbox_pred = rospy.wait_for_message('/image_processor/output', BoundingBoxes)
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
            print(output)
            st = ''
            for item in output:
                st+= str(item)+' '
            return st
        except:
            return "False -1 -1 -1 -1 -1.0 -1 -1 -1 -1"
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
    def img_callback(self,msg):
        bridge = CvBridge()
        self.img_header = msg.header
        self.img = bridge.compressed_imgmsg_to_cv2(msg,"bgr8")
        #img = img[:, 80:-80]
        #self.img = cv2.resize(self.img, (256, 256), interpolation = cv2.INTER_AREA)


if __name__ == '__main__':
    
    # parse arguments
    CLI = argparse.ArgumentParser()
    CLI.add_argument("--dummy", action="store_true") 
    CLI.add_argument("--static", action="store_true") 
    args = CLI.parse_args()
    service = call_robot_srv(args)
    rospy.spin()
