#include <iostream>
#include <string>

#include "yolo_detector.h"
#include <ros/ros.h>


int main (int argc, char **argv) {
    ros::init(argc, argv, "darknet_ros");
    ros::NodeHandle nodeHandle("~");
    darknet_ros::YoloDetector yoloDetector(nodeHandle);
    ros::spin();
    return 0;
}