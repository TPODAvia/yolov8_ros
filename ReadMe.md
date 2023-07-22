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
* ``` /yolov8/ObjectLocation ``` - publishes location of detected objects **with respect to camera frame** , along with class and probability of detected objects
* ``` /yolo_visualization ``` - can be used for visualizing results of Yolo object detection in rviz

## Test
Place trained weights and txt file having the names of classes in model directory of yolov8_ros package ( i.e. yolov8_ros/model/ ) and give their path in launch file present in sim_yolo package

```s
roslaunch sim_yolo sim_yolo_demo.launch
```
Now test your model by bringing objects in gazebo in front of camera of robot/tank 

![Screenshot from 2023-07-19 11-29-09](https://github.com/Vamsi-IITI/yolov8_ros/assets/92263050/0cecf59b-d5a3-4c39-9076-9156f25062b9)

![Screenshot from 2023-07-19 11-27-17](https://github.com/Vamsi-IITI/yolov8_ros/assets/92263050/54d7a506-70d9-430a-94be-dd4db889f9f2)

## Sample code for transforming object location from wrt. camera frame to world frame 
```
#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point
from yolov8_ros_msgs.msg import  ObjectLocations

# Reference taken from --> tf2_ros_example.py: example showing how to use tf2_ros API
# Link : https://gist.github.com/ravijo/cb560eeb1b514a13dc899aef5e6300c0

class ObjectLocationConverterNode:
    def __init__(self):
        rospy.init_node('object_location_converter_node')

        # Subscribe to the ObjectLocation topic
        rospy.Subscriber('/yolov8/ObjectLocation', ObjectLocations, self.object_location_callback)

        # Publish the object location in the world frame
        # self.world_frame_pub = rospy.Publisher('/Target_Position', PointStamped, queue_size=1)

        # define source and target frame
        self.source_frame = "camera_link"
        self.target_frame = "base_link"

    def transform_point(self, transformation, point_wrt_source):

        point_wrt_target = tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source),transformation).point

        return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]

    def get_transformation(self, source_frame, target_frame,tf_cache_duration=2.0):

        tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
        tf2_ros.TransformListener(tf_buffer)

        # get the tf at first available time
        try:
            # Get the transformation from the camera frame to the desired world frame (or robot arm base frame)
            # Setting rospy.Time(0) gives latest transform. Timeout = 2 secs
            transformation = tf_buffer.lookup_transform(target_frame,source_frame, rospy.Time(0), rospy.Duration(2))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Unable to find the transformation from %s to %s' % (source_frame, target_frame) )

        return transformation

    def object_location_callback(self, object_location_msg):
        
        # Check if any objects are detected
        if len(object_location_msg.object_location) == 0:
            rospy.loginfo("No objects detected.")
            return

        # Find the object with the highest probability
        highest_prob_object = max(object_location_msg.object_location, key=lambda x: x.probability)

        # Get the object location in the camera frame
        object_location_cam = Point(highest_prob_object.x, highest_prob_object.y, highest_prob_object.z)
        print('Object location wrt camera frame: %s' % object_location_cam)

        transformation = self.get_transformation(self.source_frame, self.target_frame)
        point_wrt_target = self.transform_point(transformation, object_location_cam)
        print('Object location wrt world frame: %s' % point_wrt_target)
        
def main():
    converter_node = ObjectLocationConverterNode()
    rospy.spin()

if __name__ == '__main__':
    main()
```
