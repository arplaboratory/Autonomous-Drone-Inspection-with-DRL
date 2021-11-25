# use system python2
# use the command line below before running this script
# export ROS_MASTER_URI=http://128.238.39.130:11311
# example: python test.py --command [x,y,z,yaw] --topic /hires/image_raw/compressed --filename image.png
# add --dummy to get black 256*256*3 image
import rospy
import argparse
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
if __name__ == '__main__':
    # parse arguments
    CLI = argparse.ArgumentParser()
    CLI.add_argument("--dummy", action="store_true")
    CLI.add_argument("--command",nargs ="*",type = float,default=[0,0,0,0])
    CLI.add_argument("--topic",type=str,default="/hires/image_raw/compressed")
    CLI.add_argument("--filename",type=str,default="image.png")
    args = CLI.parse_args()
    # dummy execution return black image
    if args.dummy:
        x = np.zeros((256,265,3),np.uint8)
        cv2.imwrite(args.filename, x)
        sys.exit(0)
    # start ros
    rospy.init_node('image_sub')
    rospy.loginfo('image_sub node started')
    # execute command
    # save image
    bridge = CvBridge()
    msg = rospy.wait_for_message(args.topic, CompressedImage)
    img  = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    img  = img[:,80:-80]
    img  = cv2.resize(img, (256,256), interpolation = cv2.INTER_AREA)
    cv2.imwrite(args.filename, img)
    print("IMAGE DONE")
