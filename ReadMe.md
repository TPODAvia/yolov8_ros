# yolov8_ros

ROS 1 wrapper for [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) to perform object detection.

Tested on Ubuntu 20.04 and ROS Noetic

## Dependencies
```s
pip install ultralytics
```

## Installation
```s
cd ~/catkin_ws/src
git clone https://github.com/Vamsi-IITI/yolov8_ros.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## Information
* ``` /yolov8/BoundingBoxes ``` - publishes bounding box related information (xmin, xmax, ymin, ymax) along with class and probability of detected objects
* ``` /yolov8/DepthPoints ``` - publishes depth/distance of center of detected objects from camera. It also publishes (x,y) coordinates of absolute center and offset center of bounding box , along with class and probability
* ``` /yolov8/ObjectLocation ``` - publishes location of detected objects with respect to camera frame , along with class and probability of detected objects
* ``` /yolo_visualization ``` - can be used for visualizing results of Yolo object detection in rviz

## Test
Place trained weights and txt file having the names of classes in model directory of yolov8_ros package ( i.e. yolov8_ros/model/ ) and give their path in launch file present in sim_yolo package

```s
roslaunch sim_yolo sim_yolo_demo.launch
```
Now test your model by bringing objects in gazebo in front of camera of robot/tank 

![Screenshot from 2023-07-19 11-29-09](https://github.com/Vamsi-IITI/yolov8_ros/assets/92263050/0cecf59b-d5a3-4c39-9076-9156f25062b9)

![Screenshot from 2023-07-19 11-27-17](https://github.com/Vamsi-IITI/yolov8_ros/assets/92263050/54d7a506-70d9-430a-94be-dd4db889f9f2)

