#!/usr/bin/env python3

import rospy
import rospkg
import cv_bridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import cv2
import torch
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image as PILImage

from ultralytics import YOLO
from shapely import Polygon


import os
import io

from get_poi import PepperPeduncle



"""
roslaunch realsense2_camera rs_aligned_depth.launch  filters:=pointcloud
"""

class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node', anonymous=True)
        self.bridge = cv_bridge.CvBridge()
        self.img_width = None
        self.img_height = None

        rospack = rospkg.RosPack()
        package_name = 'fvd_ws'
        package_path = rospack.get_path(package_name)

        # Define the YOLO model
        self.yolo = YOLO(
            package_path+'/weights/iowa_train_3.pt')
        
        self.image_subscriber = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.image_callback, queue_size=1)
        self.image_publisher = rospy.Publisher(
            '/pepper_yolo_results', Image, queue_size=1)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            if cv_image is not None:
                _ = self.run_yolo(cv_image)

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("Error converting from image message: {}".format(e))
            return

    def run_yolo(self, image):

        results_both = self.yolo(image)
        # Visualize the results on the frame
        annotated_frame = results_both[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLOv8 Tracking", annotated_frame)
        cv2.imshow("YOLOv8 Tracking orig", image)

        return image

if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# names: {0: 'bell-pepper-green', 1: 'bell-pepper-peduncle'}
