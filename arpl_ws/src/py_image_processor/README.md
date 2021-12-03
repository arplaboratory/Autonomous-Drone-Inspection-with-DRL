# py_image_processor

## Overview
The  `py_image_processor` package is a python implementation of data collection (for 3D detection / 6D pose task) in Vicon Motion Capture System. The package takes in the odometry messages from Vicon and the image messages from a front-camera on a quadrotor. Given the odometry of target object (can be a quadrotor or any object registered in Vicon system) and the quadrotor with camera in world frame, the center of target object and the 3D bounding box of the target object can be projected into camera frame as ground-truth.

## Install
The software is tested on Ubuntu 16.04 with ROS Kinetic.

First, install [ROS Driver for Motion Capture Systems](https://github.com/arplaboratory/motion_capture_system) in a ROS workspace.

Second, clone this package in the same ROS workspace.

```
git clone https://github.com/greendddd/py_image_processor.git
```

Finally, build the package.

```
catkin build py_image_processor
```

## Running

After install the package correctly, 

- Change the topic names to subscribe (should use same topic names defined in your own motion_capture_system/mocap_vicon/launch/vicon.launch ).
- Change the desired output (center of target object / 2D bounding box of target object / 3D bounding box of target object)

- Change the output path to save the data to your preferred location.

- run the script

  ```
  rosrun py_image_processor image_processor.py
  ```

## ROS nodes

### `py_image_processor` node



****Subscribed Topics****

`/vicon/{object_name}/odom` (`nav_msgs/Odometry`)



Odometry of the object in Vicon world frame



`/hires/image_raw` (`sensor_msgs/Image`)



raw image from front-camera on a quadrotor 

Notes: for faster image recording, we use yuv422 encoding on the quadrotor and change it to BGR encoding (for OpenCV) when processing the raw data.



