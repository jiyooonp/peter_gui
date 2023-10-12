#!/usr/bin/env python3

import rospy
import rospkg
import cv_bridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose
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

from get_poi import PepperPeduncle, PepperFruit, Pepper
from match_peppers_util import match_pepper_fruit_peduncle


"""
roslaunch realsense2_camera rs_aligned_depth.launch  filters:=pointcloud
"""

class PerceptionNode:

    def __init__(self):

        rospy.init_node('perception_node', anonymous=True)
        self.bridge = cv_bridge.CvBridge()
        self.depth_window = 2  # 5 x 5 window (i-2 to i+2)
        self.camera_matrix = None
        self.img_width = 640
        self.img_height = 480

        rospack = rospkg.RosPack()
        package_name = 'perception_refactor'
        package_path = rospack.get_path(package_name)

        # Define the YOLO model
        self.yolo = YOLO(
            package_path+'/weights/levelb_1.pt')
        
        # Make marker for visualization
        self.peduncle_marker = self.make_marker()
        self.pepper_marker = self.make_marker()

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
        
        # POI publisher
        self.poi_publisher = rospy.Publisher('/perception/peduncle/poi', Pose, queue_size=10)

        self.pepper_center = None
        self.peduncle_center = None
        self.depth_image = None

        self.peduncle_offset = 0.0       
        
        self.detection_void_count = 0
        self.last_peduncle_center = Point()
        self.last_pepper_center = Point()

        # store the results of YOLO
        self.fruit_count = 0
        self.peduncle_count = 0
        self.pepper_count = 0
        self.peduncle_detections = dict()
        self.pepper_detections = dict()
        self.fruit_detections = dict()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            if cv_image is not None:
                _ = self.run_yolo(cv_image)
                pepper_fruit_peduncle_match = match_pepper_fruit_peduncle(self.fruit_detections, self.peduncle_detections)

                for (pfn, ppn), _ in pepper_fruit_peduncle_match:
                    if ppn == -1:
                        continue
                    else:
                        pepper = Pepper(self.pepper_count, pfn, ppn)
                        pepper.pepper_fruit = self.fruit_detections[pfn]
                        pepper.pepper_fruit.parent_pepper = self.pepper_count
                        pepper.pepper_peduncle = self.peduncle_detections[ppn]
                        pepper.pepper_peduncle.parent_pepper = self.pepper_count
                        self.pepper_detections[self.pepper_count] = pepper
                        self.pepper_count += 1
                self.fruit_detections = dict()
                self.peduncle_detections = dict()
                print("pepper_fruit_peduncle_match\n",self.pepper_detections)

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("Error converting from image message: {}".format(e))
            return

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

        if result.boxes.data.size(0) != 0: # if there is a detection

            for i in range(result.masks.masks.size(0)): # for each mask
                segment = result.masks.segments[i] # only boundary of mask
                mask = result.masks.masks[i]       # mask with 1s and 0s
                box = result.boxes.xyxy[i]  
                cls = result.boxes.cls[i] # 0 is pepper, 1 is peduncle

                if cls == 0: # it is a pepper
                    pepper_detection = PepperFruit(self.fruit_count)

                    # TODO change (this is just a placeholder)
                    xywh = result.boxes.xywh 
                    xywh = xywh[0].cpu().numpy()
                    # Switch from YOLO axes to NumPy axes
                    xywh[0], xywh[1] = xywh[1], xywh[0]

                    pepper_detection.xywh = xywh
                    self.fruit_detections[self.fruit_count] = pepper_detection
                    self.fruit_count+= 1
                    image = self.visualize_result(image, segment)
                    

                    # These are in NumPy axes
                    self.pepper_center.x = int((box[1] + box[3]) / 2)
                    self.pepper_center.y = int((box[0] + box[2]) / 2)

                    self.peduncle_offset = int((box[3] - box[1]) / 3)
                    self.pepper_center.x -= self.peduncle_offset

                    self.pepper_center.z = 0 #self.get_depth(int(self.pepper_center.x), int(self.pepper_center.y))

                    # X, Y, Z in RS axes
                    X, Y, Z = 0, 0, 0 #self.get_3D_coords(
                        # self.peduncle_center.x, self.peduncle_center.y, self.peduncle_center.z)

                    self.pepper_marker.points.append(Point(X, Y, Z))
                    self.pepper_marker.header.stamp = rospy.Time.now()
                    self.pepper_marker_publisher.publish(self.pepper_marker)

                    self.last_pepper_center = self.pepper_center

                else: # it is a peduncle
                    
                    # TODO add pepper xywh
                    peduncle_detection = PepperPeduncle(self.peduncle_count, np.array(mask.cpu()))
                    xywh = result.boxes.xywh[i].cpu().numpy()
                    # Switch from YOLO axes to NumPy axes
                    xywh[0], xywh[1] = xywh[1], xywh[0]

                    peduncle_detection.xywh = xywh
                    poi_x, poi_y = peduncle_detection.set_point_of_interaction()

                    self.peduncle_detections[peduncle_count] = peduncle_detection
                    peduncle_count += 1
                                        
                    # These are in NumPy axes
                    self.peduncle_center.x = poi_x
                    self.peduncle_center.y = poi_y
                
                    image = self.visualize_result(image, segment, self.peduncle_center)

                    self.peduncle_center.z = 0 # self.get_depth(int(self.peduncle_center.x), int(self.peduncle_center.y))
                    
                    # X, Y, Z in RS axes
                    X, Y, Z = 0, 0, 0 #self.get_3D_coords(
                        # self.peduncle_center.x, self.peduncle_center.y, self.peduncle_center.z)

                    self.peduncle_marker.points.append(Point(X, Y, Z))
                    self.peduncle_marker.header.stamp = rospy.Time.now()
                    self.peduncle_marker_publisher.publish(self.peduncle_marker)

                    self.box_size = (box[2] - box[0]) * (box[3] - box[1])

                    self.last_peduncle_center = self.peduncle_center

                    if self.box_size > 5000:
                        self.go_straight = True

                    # get the POI in the base_link frame TODO Ishu
                    poi_in_base_link = Pose()
                    poi_in_base_link.position.x = 0
                    poi_in_base_link.position.y = 0
                    poi_in_base_link.position.z = 0
                    self.poi_publisher.publish(poi_in_base_link)
            
        else:
            self.detection_void_count += 1
            print("no detection!!")
            if self.detection_void_count > 100:
                self.peduncle_center.y = self.last_peduncle_center.y
                self.peduncle_center.x = 240
                self.pepper_center.y = self.last_pepper_center.y
                self.pepper_center.x = 240
                self.detection_void_count = 0

        # self.pepper_center_publisher.publish(self.pepper_center)
        # self.peduncle_center_publisher.publish(self.peduncle_center)


        try:
            image_msg_bb = self.bridge.cv2_to_imgmsg(image, "rgb8")
            self.image_publisher.publish(image_msg_bb)

        except cv_bridge.CvBridgeError as e:
            rospy.logerr(
                "Error converting back to image message: {}".format(e))
            return

        return image

    def depth_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')

        except cv_bridge.CvBridgeError as e:
            rospy.logerr(
                "Error converting from depth image message: {}".format(e))
            
    def get_depth(self, x, y):
        top_x = max(0, x - self.depth_window)
        bottom_x = min(self.img_height, x + self.depth_window)
    
        left_y = max(0, y - self.depth_window)
        right_y = min(self.img_width, y + self.depth_window)

        depth_values = self.depth_image[top_x:bottom_x, left_y:right_y]
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
        """
        Input: NumPy frame: (x, y) pixels and z depth in meters
        Output: RealSense frame: X, Y, Z coordinates in meters
        """
        # Switch from NumPy axes to RealSense axes
        x, y = y, x

        Z = z
        X = (x - self.camera_matrix[0, 2]) * Z / self.camera_matrix[0, 0]
        Y = (y - self.camera_matrix[1, 2]) * Z / self.camera_matrix[1, 1]
        return X, Y, Z
    
    def visualize_result(self, image, segment, poi=None, color=(100, 0, 125, 0.1)):
        mask_coords = (segment @ np.array([[self.img_width, 0], [0, self.img_height]])).astype(int)
        image = cv2.fillPoly(image, pts=[mask_coords], color=color)
        if poi is not None:
            image = cv2.circle(image, (int(poi.y), int(poi.x)), 5, (0, 0, 255), -1)
        return image
    def make_marker(self, marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.05, y=0.05):
        marker = Marker()
        marker.type = marker_type
        marker.header.frame_id = frame_id
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.scale.x = x
        marker.scale.y = y
        return marker

if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# names: {0: 'bell-pepper-green', 1: 'bell-pepper-peduncle'}
