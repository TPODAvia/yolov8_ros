# yolov8_ros

ROS 1 wrapper for [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) to perform object detection.

Tested on Ubuntu 20.04 and ROS Noetic

## Installation
```
cd ~/ros_ws/src
git clone https://github.com/mgonzs13/yolov8_ros.git
pip3 install -r yolov8_ros/requirements.txt
cd ~/ros_ws
catkin_make
sudo apt-get install ros-noetic-usb-cam
```

## Usage
```
roslaunch yolov8_ros yolov8.launch
```
or
```
rosrun yolov8_ros yolo_ros.py
rosrun usb_cam usb_cam_node
```

## Use own weights and datasets

in the development
