#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import os
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image as PILImage
from shapely import Polygon
import cv_bridge
import io
import rospkg
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from get_poi import PepperPeduncle



"""
roslaunch realsense2_camera rs_aligned_depth.launch  filters:=pointcloud
"""

class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node', anonymous=True)
        self.bridge = CvBridge()
        self.depth_window = 2  # 5 x 5 window (i-2 to i+2)
        self.camera_matrix = None
        self.img_width = None
        self.img_height = None

        curr_path = os.getcwd()

        # Define the YOLO model
        rospack = rospkg.RosPack()
        package_name = 'perception_refactor'
        package_path = rospack.get_path(package_name)

        self.yolo = YOLO(
            package_path+'/weights/iowa_train_3.pt')
        
        # Make marker for visualization
        self.peduncle_marker = Marker()
        self.peduncle_marker.type = 8
        self.peduncle_marker.header.frame_id = "camera_color_optical_frame"
        self.peduncle_marker.color.r = 1.0
        self.peduncle_marker.color.g = 0.0
        self.peduncle_marker.color.b = 0.0
        self.peduncle_marker.color.a = 1.0
        self.peduncle_marker.scale.x = 0.05
        self.peduncle_marker.scale.y = 0.05

        self.pepper_marker = Marker()
        self.pepper_marker.type = 8
        self.pepper_marker.header.frame_id = "camera_color_optical_frame"
        self.pepper_marker.color.r = 1.0
        self.pepper_marker.color.g = 0.0
        self.pepper_marker.color.b = 0.0
        self.pepper_marker.color.a = 1.0
        self.pepper_marker.scale.x = 0.05
        self.pepper_marker.scale.y = 0.05

        self.go_straight = False

        # Define the RealSense image subscriber
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        self.peduncle_marker_publisher = rospy.Publisher("/visualization_peduncle_marker", Marker, queue_size=1)
        self.pepper_marker_publisher = rospy.Publisher("/visualization_pepper_marker", Marker, queue_size=1)

        self.depth_subscriber = rospy.Subscriber(
            '/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=1)
        self.image_subscriber = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.image_callback, queue_size=1)
        self.image_publisher = rospy.Publisher(
            '/pepper_yolo_results', Image, queue_size=1)
        self.pepper_center_publisher = rospy.Publisher(
            '/pepper_center', Point, queue_size=1)
        self.peduncle_center_publisher = rospy.Publisher(
            '/peduncle_center', Point, queue_size=1)
        
        self.peduncle_box_size_publisher = rospy.Publisher(
            '/peduncle_box_size', String, queue_size=1)

        self.pepper_center = None
        self.peduncle_center = None
        self.depth_image = None

        self.peduncle_offset = 0.0       
        
        self.detection_void_count = 0
        self.last_peduncle_center = Point()
        self.last_pepper_center = Point()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            if cv_image is not None:
                _ = self.run_yolo(cv_image)

        except CvBridgeError as e:
            rospy.logerr("Error converting from image message: {}".format(e))
            return

        # Run YOLO on the image

    def run_yolo(self, image):

        results_both = self.yolo(image)

        self.peduncle_dict = dict()
        peduncle_count = 0

        self.pepper_center = Point()
        self.pepper_center.x = -1 # Assume there are no detections initially
        self.pepper_center.y = -1
        self.pepper_center.z = 0

        self.peduncle_center = Point()
        self.peduncle_center.x = -1 # self.img_width/2
        self.peduncle_center.y = -1 # self.img_height/2
        self.peduncle_center.z = 0

        self.pepper_marker.points = []
        self.peduncle_marker.points = []

        result = results_both[0] # only take the first image because there is only one image

        if result.boxes.boxes.size(0) != 0: # if there is a detection

            for i in range(result.masks.masks.size(0)): # for each mask
                segment = result.masks.segments[i] # only boundary of mask
                mask = result.masks.masks[i]       # mask with 1s and 0s
                box = result.boxes.xyxy[i]  
                cls = result.boxes.cls[i] # 0 is pepper, 1 is peduncle

                if cls == 0: # it is a pepper
                    mask_coords = (segment @ np.array([[self.img_width, 0], [0, self.img_height]])).astype(int)
                    image = cv2.fillPoly(image, pts=[mask_coords], color=(100, 0, 125, 0.1))

                    # These are in RealSense coordinate system
                    self.pepper_center.x = int((box[0] + box[2]) / 2)
                    self.pepper_center.y = int((box[1] + box[3]) / 2)

                    self.peduncle_offset = int((box[3] - box[1]) / 3)
                    self.pepper_center.y -= self.peduncle_offset

                    self.pepper_center.z = self.get_depth(int(self.pepper_center.x), int(self.pepper_center.y))

                    # X, Y, Z in RS frame
                    X, Y, Z = self.get_3D_coords(
                        self.pepper_center.x, self.pepper_center.y, self.pepper_center.z)

                    self.pepper_marker.points.append(Point(X, Y, Z))
                    self.pepper_marker.header.stamp = rospy.Time.now()
                    self.pepper_marker_publisher.publish(self.pepper_marker)

                    self.last_pepper_center = self.pepper_center

                else: # it is a peduncle
                    peduncle = PepperPeduncle(i, np.array(mask.cpu()))
                    poi_x, poi_y = peduncle.set_point_of_interaction()

                    self.peduncle_dict[peduncle_count] = peduncle
                    peduncle_count += 1
                                        
                    mask_coords = (segment @ np.array([[self.img_width, 0], [0, self.img_height]])).astype(int)
                    image = cv2.fillPoly(image, pts=[mask_coords], color=(0, 0, 255, 0.1))

                    # These are in RealSense coordinate system
                    self.peduncle_center.x = poi_y
                    self.peduncle_center.y = poi_x

                    self.peduncle_center.z = self.get_depth(int(self.peduncle_center.x), int(self.peduncle_center.y))
                    
                    # RS frame
                    X, Y, Z = self.get_3D_coords(
                        self.peduncle_center.x, self.peduncle_center.y, self.peduncle_center.z)

                    self.peduncle_marker.points.append(Point(X, Y, Z))
                    self.peduncle_marker.header.stamp = rospy.Time.now()
                    self.peduncle_marker_publisher.publish(self.peduncle_marker)

                    self.box_size = (box[2] - box[0]) * (box[3] - box[1])

                    self.last_peduncle_center = self.peduncle_center

                    if self.box_size > 5000:
                        self.go_straight = True
        else:
            self.detection_void_count += 1
            if self.detection_void_count > 10:
                self.peduncle_center = self.last_peduncle_center
                self.pepper_center = self.last_pepper_center
                self.detection_void_count = 0

        self.pepper_center_publisher.publish(self.pepper_center)
        self.peduncle_center_publisher.publish(self.peduncle_center)


        try:
            image_msg_bb = self.bridge.cv2_to_imgmsg(image, "rgb8")
            self.image_publisher.publish(image_msg_bb)
            print("published image")

        except CvBridgeError as e:
            rospy.logerr(
                "Error converting back to image message: {}".format(e))
            return

        return image

    def depth_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')

        except CvBridgeError as e:
            rospy.logerr(
                "Error converting from depth image message: {}".format(e))
            
    def get_depth(self, x, y):
        left_x = max(0, x - self.depth_window)
        right_x = min(self.img_width, x + self.depth_window)
        top_y = max(0, y - self.depth_window)
        bottom_y = min(self.img_height, y + self.depth_window)

        depth_values = self.depth_image[top_y:bottom_y, left_x:right_x]
        depth_values = depth_values.flatten()
        depth = 0.001*np.median(depth_values)

        depth = 0 if np.isnan(depth) else depth

        return depth
    

    def camera_info_callback(self, msg):
        # Store the camera matrix
        self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.img_height = msg.height
        self.img_width = msg.width
        self.camera_info_sub.unregister()

    def get_3D_coords(self, x, y, z):
        # Get the 3D coordinates of the pixel in the RS frame?TODO
        Z = z
        X = (x - self.camera_matrix[0, 2]) * Z / self.camera_matrix[0, 0]
        Y = (y - self.camera_matrix[1, 2]) * Z / self.camera_matrix[1, 1]
        return X, Y, Z


if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




# names: {0: 'bell-pepper-green', 1: 'bell-pepper-peduncle'}
