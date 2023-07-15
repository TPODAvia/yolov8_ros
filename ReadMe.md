# yolov8_ros

ROS 1 wrapper for [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) to perform object detection.

Tested on Ubuntu 20.04 and ROS Noetic

## Installation
```s
cd ~/ros_ws/src
git clone https://github.com/Vamsi-IITI/yolov8_ros.git
cd ~/ros_ws
catkin_make
```

## Dependencies
```s
pip install ultralytics
```

## Usage
Place trained weights and txt file having the names of classes in model directory of yolov8_ros package ( i.e. yolov8_ros/model/ ) and give their path in launch file present in sim_yolo package

```s
roslaunch sim_yolo sim_yolo_demo.launch
```
Now test your model by bringing objects in gazebo in front of camera of robot/tank 

![Screenshot from 2023-07-14 15-18-06](https://github.com/Vamsi-IITI/yolov8_ros/assets/92263050/12047eb0-236f-4c0a-bb98-b3952a402750)
