# YOLO V3 ros package

**Author: Yichen Hu, yichen.hu@nyu.edu**
**Affiliation: [Agile Robotics and Perception Lab](https://wp.nyu.edu/arpl/), New York University**

More details: 
- http://pjreddie.com/darknet/yolo/

## Introduction

This ros package is a darknet-updated version of the original [darknet_ros](https://github.com/leggedrobotics/darknet_ros) rospackage by Robotic Systems Lab, ETH Zurich. We update the darknet framework on the base of [AlexeyAB's darknet](https://github.com/AlexeyAB/darknet). You can see details and improvements in their github repositories.

## Requirements
- CUDA 10.0
- OpenCV >= 2.4
- cuDNN >= 7.0 for CUDA 10.0
- ubuntu 18.04, ROS melodic

## Building
In order to install darknet_ros, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

## Basic Usage
In order to get YOLO ROS: Real-Time Object Detection for ROS to run with your robot, you will need to adapt a few parameters. It is the easiest if duplicate and adapt all the parameter files that you need to change from the `darkned_ros` package. These are specifically the parameter files in `config` and the launch file from the `launch` folder.
```sh
$ roslaunch darknet_ros darknet_ros.launch
```

## Nodes

### Node: darknet_ros

This is the main YOLO ROS: Real-Time Object Detection for ROS node. It uses the camera measurements to detect pre-learned objects in the frames.

### ROS related parameters

You can change the names and other parameters of the publishers, subscribers and actions inside `darkned_ros/config/ros.yaml`.

#### Subscribed Topics

* **`/camera_reading`** ([sensor_msgs/Image])

    The camera measurements.

#### Published Topics

* **`object_detector`** ([std_msgs::Int8])

    Publishes the number of detected objects.

* **`bounding_boxes`** ([darknet_ros_msgs::BoundingBoxes])

    Publishes an array of bounding boxes that gives information of the position and size of the bounding box in pixel coordinates.

* **`detection_image`** ([sensor_msgs::Image])

    Publishes an image of the detection image including the bounding boxes. Notice that image will be publish only when a node is subsribing this rostopic.

#### Actions

* **`camera_reading`** ([sensor_msgs::Image])

    Sends an action with an image and the result is an array of bounding boxes.

### Detection related parameters

You can change the parameters that are related to the detection by adding a new config file that looks similar to `darkned_ros/config/yolo.yaml`.

* **`image_view/enable_opencv`** (bool)

    Enable or disable the open cv view of the detection image including the bounding boxes.

* **`image_view/wait_key_delay`** (int)

    Wait key delay in ms of the open cv window.

* **`yolo_model/config_file/name`** (string)

    Name of the cfg file of the network that is used for detection. The code searches for this name inside `darkned_ros/yolo_network_config/cfg/`.

* **`yolo_model/weight_file/name`** (string)

    Name of the weights file of the network that is used for detection. The code searches for this name inside `darkned_ros/yolo_network_config/weights/`.

* **`yolo_model/threshold/value`** (float)

    Threshold of the detection algorithm. It is defined between 0 and 1.

* **`yolo_model/detection_classes/names`** (array of strings)

    Detection names of the network used by the cfg and weights file inside `darkned_ros/yolo_network_config/`.
