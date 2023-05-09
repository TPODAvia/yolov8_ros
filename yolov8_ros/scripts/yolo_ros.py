#!/usr/bin/env python3

import cv2
import torch
import numpy

import rospy
import sys

from time import time
from cv_bridge import CvBridge
from ultralytics import YOLO

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D,  BoundingBox2D
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes

from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D

def create_header():
    h = Header()
    h.stamp = rospy.Time.now()
    return h

class yolo_class:

    def __init__(self, weights_path,classes_path,img_topic,bbox_topic,queue_size,visualize):

        self._class_to_color = {}
        self.cv_bridge = CvBridge()
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("Using Device: ", self.device)
        
        self.yolo = YOLO("yolov8n.pt")
        self.yolo.fuse()

        self.img_subscriber = rospy.Subscriber(img_topic, Image, self.process_img_msg)

        self.boundingbox_publisher = rospy.Publisher(bbox_topic, BoundingBox2D, queue_size=queue_size)
        
        self.visualize = visualize
        if self.visualize == True:
            self.visualization_publisher = rospy.Publisher("/yolo_visualization", Image, queue_size=queue_size)

    
    def process_img_msg(self, img_msg: Image):

        start_time = time()
        np_img_orig = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        results = self.yolo(np_img_orig)

        frame = results[0].plot()
        boxes = results[0].boxes


        # IDK how to stack bbox together :(
        bboxes = []
        for i in range(len(boxes)):
            for j in (0,1,2,3):
                box = boxes[i].xyxy[0][j]
                bboxes.append(box.item())
                print(boxes[0].xyxy)
                print(bboxes)

        try:
            
            x1 = boxes[0].xyxy[0][0].item()
            y1 = boxes[0].xyxy[0][1].item()
            x2 = boxes[0].xyxy[0][2].item()
            y2 = boxes[0].xyxy[0][3].item()

            w = int(round(x2 - x1))
            h = int(round(y2 - y1))
            cx = int(round(x1 + w / 2))
            cy = int(round(y1 + h / 2))
            offset_cx = 640/2 - cx
            offset_cy = 480/2 - cy

            bounding_boxes = BoundingBox2D()

            # header = create_header()
            # bounding_boxes.header = header
            # bounding_boxes


            bounding_boxes.size_x = w
            bounding_boxes.size_y = h
            
            # offset from center in pixels
            bounding_boxes.center = Pose2D()
            bounding_boxes.center.x = offset_cx
            bounding_boxes.center.y = offset_cy

            # print(bounding_boxes)
            self.boundingbox_publisher.publish(bounding_boxes)  
        except:   
            pass

        self.dectshow(results, 480, 640)


        if self.visualize == True:
            end_time = time()
            fps = 1/numpy.round(end_time - start_time, 2)

            cv2.putText(frame, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)

            vis_msg = self.cv_bridge.cv2_to_imgmsg(frame)
            self.visualization_publisher.publish(vis_msg)

    def dectshow(self, results, height, width):

        self.frame = results[0].plot()
        print(str(results[0].speed['inference']))
        fps = 1000.0/ results[0].speed['inference']
        cv2.putText(self.frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        for result in results[0].boxes:
            boundingBox = BoundingBox()
            boundingBox.xmin = numpy.int64(result.xyxy[0][0].item())
            boundingBox.ymin = numpy.int64(result.xyxy[0][1].item())
            boundingBox.xmax = numpy.int64(result.xyxy[0][2].item())
            boundingBox.ymax = numpy.int64(result.xyxy[0][3].item())
            boundingBox.Class = results[0].names[result.cls.item()]
            boundingBox.probability = result.conf.item()
            self.boundingBoxes.bounding_boxes.append(boundingBox)
        self.position_pub.publish(self.boundingBoxes)
        self.publish_image(self.frame, height, width)

        if self.visualize :
            cv2.imshow('YOLOv8', self.frame)

    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = numpy.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main(args):

    rospy.init_node('yolov8_node', anonymous=True)

    weights_path = rospy.get_param("~weights_path", "")
    classes_path = rospy.get_param("~classes_path", "")
    img_topic = rospy.get_param("~img_topic", "/usb_cam/image_raw")
    bbox_topic = rospy.get_param("~center_offset_topic", "/yolo_bbox" )
    queue_size = rospy.get_param("~queue_size", 1)
    visualize = rospy.get_param("~visualize", False) # IDK why but open cv visualisation won't work :(
    
    
    yolo_class(weights_path,classes_path,img_topic,bbox_topic,queue_size,visualize)


    rospy.loginfo("YOLOv8 initialization complete")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)