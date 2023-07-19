cmake_minimum_required(VERSION 3.0.2)
project(yolov8_ros_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
        DIRECTORY msg
        FILES
        BoundingBox.msg
        BoundingBoxes.msg
        DepthPoint.msg
        DepthPoints.msg
        Object.msg
        ObjectLocations.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
