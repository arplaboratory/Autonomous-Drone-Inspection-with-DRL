#! /usr/bin/python2.7
import os
import sys
import time
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import *
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf
import tf.transformations as tfs
import pickle

obj_nums = 1
cam_drone_name = 'dragonfly12'
image_topic_name = '/hires/image_raw/compressed' # '/stereo/left/image_raw'
delta = [0.0,0.0,0.0]
object_name = ['dragonfly13'] 
object_size = [ # same for DragonFly2/3/5
    (-0.09, 0.09, -0.09, 0.09, -0.045, 0.045)]
#    (-0.15, 0.01, -0.09, 0.09, -0.033, 0.057)]
#    (-0.15, 0.01, -0.09, 0.09, -0.033, 0.057),
#    (-0.15, 0.01, -0.09, 0.09, -0.033, 0.057)]  
#   (-0.15, 0.01, -0.09, 0.09, -0.025, 0.065),  # DragonPro1
#   (-0.15, 0.01, -0.09, 0.09, -0.025, 0.065)]  # DragonPro2
# xmin xmax ymin ymax zmin zmax
# meter

# object_size = [
#     (-0.036392, 0.035277, -0.08356, 0.080159, -0.111481, 0.101871),
#     (-0.023678, 0.025582, -0.048457, 0.045523, -0.092318, 0.083589),
#     (-0.033504, 0.033552, -0.032167, 0.035221, -0.059196, 0.042389),
#     (-0.049807, 0.047208, -0.035672, 0.030698, -0.083758, 0.107416),
#     (-0.070361, 0.049325, -0.083416, 0.087344, -0.018994, 0.019943)]
object_color = [(255, 0, 0),
                (0, 255, 0),
                (0, 0, 255),
                (255, 255, 0),
                (0, 255, 255)]

# stero_left
# K = np.array([431.13546, 0.000000, 329.42338,
#               0.000000, 431.1209, 243.22938,
#               0.000000, 0.000000, 1.000000]).reshape((3, 3))
K = np.array([278.745789, 0, 324.056283,
             0, 279.599725, 240.812296,
              0.000000, 0.000000, 1.000000]).reshape((3, 3))
        

#R_offset = tfs.euler_matrix(0.02, -0.28, 0, 'rzyx')  # yaw pitch roll
R_offset = tfs.euler_matrix(0.02, -0.25, 0, 'rzyx')  # yaw pitch roll
R_cam_world = np.array([0, -1, 0, 0,
                        0, 0, -1, 0,
                        1, 0, 0, 0,
                        0, 0, 0, 1]).reshape(4, 4).dot(R_offset)
bridge = CvBridge()


class Processor:

    def __init__(self, data_root='.'):
        self.data_root = data_root
        self.list_cam_odom = []
        self.list_object_odom = [[], [], [], [], []]
        self.sub_pose_list = []
        self.imu = []
        self.vio = []

        func_list = [self.object1_callback, self.object2_callback, self.object3_callback] # as much as the number of objects
        for i in range(obj_nums):
            self.sub_pose_list.append(rospy.Subscriber(
                '/{}/odom'.format(object_name[i]), Odometry, func_list[i], queue_size=10))
        self.sub_cam = rospy.Subscriber(
            '/'+cam_drone_name+'/odom', Odometry, self.pose_cam_callback, queue_size=10)
        self.sub_img = rospy.Subscriber(
            image_topic_name, CompressedImage, self.img_callback, queue_size=10)
        self.pub_bbox = rospy.Publisher('image_processor/image_bbox', Image, queue_size=10)
        self.pub_center = rospy.Publisher('image_processor/objects_center', PoseArray, queue_size=10)
        self.pub_output = rospy.Publisher('image_processor/output', BoundingBoxes, queue_size=10)

        # self.sub_vio = rospy.Subscriber(
        #     '/dragonfly5/quadrotor_ukf/control_odom', Odometry, self.vio_callback, queue_size=10)
        # self.sub_imu = rospy.Subscriber(
        #     '/dragonfly5/quadrotor_ukf/imu_calib', TwistStamped, self.imu_callback, queue_size=10)

        self.tfROS = tf.TransformerROS()
        self.first = True
        self.ts_offset = 0
        self.img_count = 0

    # def vio_callback(self, msg):
    #     self.vio.append([
    #         msg.header.stamp.to_sec(),
    #         msg.pose.pose.position.x,
    #         msg.pose.pose.position.y,
    #         msg.pose.pose.position.z,
    #         msg.pose.pose.orientation.x,
    #         msg.pose.pose.orientation.y,
    #         msg.pose.pose.orientation.z,
    #         msg.pose.pose.orientation.w
    #     ])

    # def imu_callback(self, msg):
    #     self.imu.append([
    #         msg.header.stamp.to_sec(),
    #         msg.twist.angular.x,
    #         msg.twist.angular.y,
    #         msg.twist.angular.z,
    #         msg.twist.linear.x,
    #         msg.twist.linear.y,
    #         msg.twist.linear.z
    #     ])

    def pose_cam_callback(self, odom):
        self.list_cam_odom.append(odom)
        #print('odom cam_callback')

    def object1_callback(self, odom):
        self.list_object_odom[0].append(odom)
        #print('odom 1_callback')

    def object2_callback(self, odom):
        self.list_object_odom[1].append(odom)
        #print('odom 2_callback')

    def object3_callback(self, odom):
        self.list_object_odom[2].append(odom)

    # def object4_callback(self, odom):
    #     self.list_object_odom[3].append(odom)

    # def object5_callback(self, odom):
    #     self.list_object_odom[4].append(odom)

    def img_callback(self, img_msg):
        # get offset for hires camera
        # no need for stereo camera grayscale image
        # if (self.first):
        #     if len(self.list_cam_odom) == 0:
        #         return
        #     self.first = False
        #     self.ts_offset = img_msg.header.stamp.to_sec(
        #     ) - self.list_cam_odom[0].header.stamp.to_sec()
        #     print(self.ts_offset)
        #     return

        if len(self.list_cam_odom) == 0 or len(self.list_object_odom[0]) == 0:
            print('return')
            return

        # if len(self.list_cam_odom) == 0 or len(self.list_object_odom[0]) == 0 or len(self.list_object_odom[1]) == 0 or len(self.list_object_odom[2]) == 0 or len(self.list_object_odom[3]) == 0 or len(self.list_object_odom[4]) == 0:
        #     return

        # imu_dict = {'vio': np.array(self.vio),
        #             'imu': np.array(self.imu)}
        # self.vio = []
        # self.imu = []

        # print(len(self.list_cam_odom), [len(self.list_object_odom[i]) for i in range(5)])

        # offset for hires camera
        # ts_now = img_msg.header.stamp.to_sec() - self.ts_offset

        # no offset for stereo camera grayscale image
        ts_now = img_msg.header.stamp.to_sec()

        odom_index = self.search_odom_index(ts_now, self.list_cam_odom)
        cam_odom = self.list_cam_odom[odom_index]
        del self.list_cam_odom[:odom_index//2]
        objects_index = [self.search_odom_index(
            ts_now, self.list_object_odom[i]) for i in range(obj_nums)]
        objects_odom = [self.list_object_odom[i][objects_index[i]]
                        for i in range(obj_nums)]  # [odom msg for each of 5 objects]
        for i in range(obj_nums):
            del self.list_object_odom[i][:objects_index[i]//2]
        # print(objects_index)
        pose = cam_odom.pose.pose
        H_w_c = self.tfROS.fromTranslationRotation(  # cam in world
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w)
        )

        img_bbox = bridge.compressed_imgmsg_to_cv2(img_msg, 'bgr8')  
        # DragonFly4 hires camera use yuv422 encoding for faster recording
        # img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_Y422)  # yuv422 to BGR

        # DragonPro2 stereo left camera use grayscale image
        # img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        # DragonPro2/1 hires camera rotate image
        #img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        
        #img_bbox = img.copy()
        gt_objects = []
        dets = []
        dets_new = []
        for i in range(obj_nums):
            pose = objects_odom[i].pose.pose
            pose.position.x+=delta[0]
            pose.position.y+=delta[1]
            pose.position.z+=delta[0]
            H_w_o = self.tfROS.fromTranslationRotation(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w)
            )
            H_cam_obj = np.linalg.inv(H_w_c).dot(H_w_o)
            H_gt = R_cam_world.dot(H_cam_obj)
            H_gt = H_gt[:3, :]  # 3x4
            gt_objects.append(H_gt)
            # get center
            pt = K.dot(H_gt[:, -1])
            center = (int(pt[0]/pt[2]), int(pt[1]/pt[2]))
            # print(center)
            # get corner

            # special treatment! xyz from points.xyz not match with blender axis

            xmin, xmax, ymin, ymax, zmin, zmax = object_size[i]
            corner = np.array([
                (xmax, ymax, zmax, 1),
                (xmax, ymax, zmin, 1),
                (xmax, ymin, zmax, 1),
                (xmax, ymin, zmin, 1),
                (xmin, ymax, zmax, 1),
                (xmin, ymax, zmin, 1),
                (xmin, ymin, zmax, 1),
                (xmin, ymin, zmin, 1),
            ])
            corners = []
            for j in range(8):
                pt = H_cam_obj.dot(corner[j, :])
                pt = K.dot(R_cam_world[:3, :3].dot(pt[:3]))
                corner_pt = (int(pt[0]/pt[2]), int(pt[1]/pt[2]))
                corners.append(corner_pt)

            np_corners = np.array(corners)
            xmin, xmax, ymin, ymax = np.min(np_corners[:, 0]), np.max(
                np_corners[:, 0]), np.min(np_corners[:, 1]), np.max(np_corners[:, 1])

            dets.append([
                (xmin+xmax)/2, (ymin+ymax)/2,
                xmax-xmin, ymax-ymin
            ])
            dets_new.append([ xmin, ymin, xmax, ymax])
            # dets.append([center[0], center[1],
            #              (xmax-xmin), (ymax-ymin)])
            ########################################################################
            # draw center
            cv2.circle(img_bbox, center, 1, (0, 255, 255), 4)
            # draw 3D bbox
            cv2.line(img_bbox, corners[0], corners[4], object_color[i], 1)
            cv2.line(img_bbox, corners[1], corners[5], object_color[i], 1)
            cv2.line(img_bbox, corners[2], corners[6], object_color[i], 1)
            cv2.line(img_bbox, corners[3], corners[7], object_color[i], 1)
            cv2.line(img_bbox, corners[0], corners[2], object_color[i], 1)
            cv2.line(img_bbox, corners[1], corners[3], object_color[i], 1)
            cv2.line(img_bbox, corners[4], corners[6], object_color[i], 1)
            cv2.line(img_bbox, corners[5], corners[7], object_color[i], 1)
            cv2.line(img_bbox, corners[0], corners[1], object_color[i], 1)
            cv2.line(img_bbox, corners[2], corners[3], object_color[i], 1)
            cv2.line(img_bbox, corners[4], corners[5], object_color[i], 1)
            cv2.line(img_bbox, corners[6], corners[7], object_color[i], 1)

            # draw 2D bbox
            cv2.rectangle(
                img_bbox,
                (int(xmin),
                int(ymin)),
                (int(xmax),
                int(ymax)),
                object_color[i],
                1
            )
            ###################################################
            # cv2.rectangle(
            #     img_bbox,
            #     (int(center[0]-0.5*(xmax-xmin)),
            #      int(center[1]-0.5*(ymax-ymin))),
            #     (int(center[0]+0.5*(xmax-xmin)),
            #      int(center[1]+0.5*(ymax-ymin))),
            #     object_color[i],
            #     1
            # )

        ts = img_msg.header.stamp.to_sec()
        dets = np.array(dets)  # 5x4 [x,y,w,h]
        #print(dets)
        dets_new = np.array(dets_new)
        print(dets_new)
        gt_objects = np.array(gt_objects)  # 5x3x4 [R;t]
        # dic = {
        #     'ts': ts,
        #     'pose': gt_objects,
        #     'det': dets
        # }
        # with open(self.data_root+'gt/'+str(self.img_count)+'.pkl', 'wb') as ff:
        #     pickle.dump(dic, ff)
        #np.savez(self.data_root+'gt/'+str(self.img_count)+'.npz', ts=ts, pose=gt_objects, dets=dets)
        
        # save the raw and bbox image for vis
        #cv2.imwrite(self.data_root+'raw/'+str(self.img_count)+'.png', img)
        #cv2.imwrite(self.data_root+'bbox/' +
        #            str(self.img_count)+'.png', img_bbox)
        self.img_count = self.img_count + 1

        self.pub_bbox.publish(bridge.cv2_to_imgmsg(img_bbox, encoding="bgr8"))
        # print('publish')

        #ps = PoseArray()
        #ps.header = img_msg.header
        #for i in range(obj_nums):
        #    pose = Pose()
        #    pose.position.x = dets[i][0]
        #    pose.position.y = dets[i][1]
        #    ps.poses.append(pose)
        #self.pub_center.publish(ps)
        bboxes = BoundingBoxes()
        bboxes.header = img_msg.header
        bboxes.header.frame_id = "groundtruth"
        bboxes.image_header = img_msg.header
        bbox = BoundingBox()
        #if not(xmin <0 or ymin<0 or xmax>480 or ymax>640):
        if True:
            bboxes = BoundingBoxes()
            bboxes.header = img_msg.header
            bboxes.image_header = img_msg.header
            bbox = BoundingBox()
            bbox.Class = 'drone'
            bbox.probability = 1.0
            bbox.xmin = xmin
            bbox.ymin = ymin
            bbox.xmax = xmax
            bbox.ymax = ymax
            bboxes.bounding_boxes = [bbox]
            self.pub_output.publish(bboxes)

    def search_odom_index(self, ts_now, list_odom):
        for i, odom in enumerate(list_odom):
            if odom.header.stamp.to_sec() >= ts_now:
                return i
        return -1


if __name__ == '__main__':
    # bagname = sys.argv[-1]
    # data_root = './'+bagname+'/'
    # try:
    #     os.mkdir(data_root)
    #     os.mkdir(data_root+'raw')
    #     os.mkdir(data_root+'bbox')
    #     os.mkdir(data_root+'gt')
        # os.mkdir(data_root+'crop')
    # except:
    #     pass

    rospy.init_node("py_image_processor")
    image_processor = Processor('.')
    rospy.spin()
