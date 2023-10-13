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

import tf2_ros
from std_msgs.msg import String, Int16
from tf.transformations import quaternion_matrix

from ultralytics import YOLO
from shapely import Polygon

import os
import io
import random

from pepper_util import PepperPeduncle, PepperFruit, Pepper
from match_peppers_util import match_pepper_fruit_peduncle

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
        self.peduncle_marker_rs =  self.make_marker(frame_id="camera_color_optical_frame")
        self.peduncle_marker_base =  self.make_marker(marker_type=8, frame_id='link_base', r= 1, g=0, b=1, a=1, x=0.02, y=0.01)
        self.pepper_marker_rs =  self.make_marker(marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.05, y=0.05)
        self.pepper_marker_base =  self.make_marker(marker_type=8, frame_id='link_base', r= 0, g=1, b=0, a=1, x=0.06, y=0.06)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publishers
        self.peduncle_marker_rs_pub = rospy.Publisher("/visualization_peduncle_marker_rs", Marker, queue_size=1)
        self.peduncle_marker_base_pub = rospy.Publisher("/visualization_peduncle_marker_base", Marker, queue_size=1)

        self.pepper_marker_rs_pub = rospy.Publisher("/visualization_pepper_marker_rs", Marker, queue_size=1)
        self.pepper_marker_base_pub = rospy.Publisher("/visualization_pepper_marker_base", Marker, queue_size=1)

        self.image_pub = rospy.Publisher('/pepper_yolo_results', Image, queue_size=1)
        self.pepper_center_pub = rospy.Publisher('/pepper_center', Point, queue_size=1)
        self.peduncle_center_pub = rospy.Publisher('/peduncle_center', Point, queue_size=1)
        self.poi_pub = rospy.Publisher('/poi', Point, queue_size=1)

        self.poi_publisher = rospy.Publisher('/perception/peduncle/poi', Pose, queue_size=10)

        # Subscribers
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1)

        self.state_sub = rospy.Subscriber('/state', Int16, self.state_callback, queue_size=1)

        
        self.poi = Point()
        self.state = None
        
        # POI publisher

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
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

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

                image = cv_image

                if self.pepper_detections != dict():
                    for i, pepper in self.pepper_detections.items():
                        rand_color = self.random_color()
                        image = self.visualize_result(cv_image, pepper.pepper_fruit.segment, color=rand_color)
                        image = self.visualize_result(cv_image, pepper.pepper_peduncle.segment, color=rand_color)

                try:
                    image_msg_bb = self.bridge.cv2_to_imgmsg(image, "rgb8")
                    self.image_pub.publish(image_msg_bb)

                except cv_bridge.CvBridgeError as e:
                    rospy.logerr(
                        "Error converting back to image message: {}".format(e))
                    
                self.pepper_detections = dict()

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("Error converting from image message: {}".format(e))
            return

    def run_yolo(self, image):

        results_both = self.yolo(image)

        self.peduncle_dict = dict()
        peduncle_count = 0

        self.pepper_center = Point() # TODO can we just do Point(x=-1, y=-1, z=0)?
        self.pepper_center.x = -1 # Assume there are no detections initially
        self.pepper_center.y = -1
        self.pepper_center.z = 0

        self.peduncle_center = Point() # TODO can we just do Point(x=-1, y=-1, z=0)?
        self.peduncle_center.x = -1 # self.img_width/2
        self.peduncle_center.y = -1 # self.img_height/2
        self.peduncle_center.z = 0

        self.pepper_marker_rs.points = []
        self.pepper_marker_base.points = []
        self.peduncle_marker_rs.points = []
        self.peduncle_marker_base.points = []

        result = results_both[0] # only take the first image because there is only one image

        if result.boxes.data.size(0) != 0: # if there is a detection

            for i in range(result.masks.data.size(0)): # for each mask
                segment = result.masks.segments[i] # only boundary of mask
                mask = result.masks.masks[i]       # mask with 1s and 0s
                box = result.boxes.xyxy[i]  
                cls = result.boxes.cls[i] # 0 is pepper, 1 is peduncle

                if cls == 0: # it is a pepper
                    pepper_detection = PepperFruit(self.fruit_count, segment=segment)
                    xywh = result.boxes.xywh # TODO change (this is just a placeholder)
                    pepper_detection.xywh = xywh[0].cpu().numpy()
                    self.fruit_detections[self.fruit_count] = pepper_detection
                    self.fruit_count+= 1

                    # These are in RealSense coordinate system
                    self.pepper_center.x = int((box[0] + box[2]) / 2)
                    self.pepper_center.y = int((box[1] + box[3]) / 2)

                    self.peduncle_offset = int((box[3] - box[1]) / 3)
                    self.pepper_center.y -= self.peduncle_offset

                    self.pepper_center.z = 0 #self.get_depth(int(self.pepper_center.x), int(self.pepper_center.y))

                    # X, Y, Z in RS frame
                    # X, Y, Z = self.get_3D_coords(
                    #     self.pepper_center.x, self.pepper_center.y, self.pepper_center.z)
                    
                    # self.pepper_marker_rs.points.append(Point(X, Y, Z))
                    # self.pepper_marker_rs.header.stamp = rospy.Time.now()
                    # self.pepper_marker_rs_pub.publish(self.pepper_marker_rs)

                    # X, Y, Z in base frame
                    # X_b, Y_b, Z_b = self.transform_to_base_frame(X, Y, Z)

                    # self.pepper_marker_base.points.append(Point(X_b, Y_b, Z_b))
                    # self.pepper_marker_base.header.stamp = rospy.Time.now()
                    # self.pepper_marker_base_pub.publish(self.pepper_marker_base)

                    # if self.state != 5:
                    #     self.poi.x = X_b
                    #     self.poi.y = Y_b
                    #     self.poi.z = Z_b

                    self.last_pepper_center = self.pepper_center

                else: # it is a peduncle
                    
                    peduncle_detection = PepperPeduncle(self.peduncle_count, np.array(mask.cpu()), segment=segment)
                    peduncle_detection.xywh = result.boxes.xywh[i].cpu().numpy()
                    poi_x, poi_y = peduncle_detection.set_point_of_interaction()

                    self.peduncle_detections[peduncle_count] = peduncle_detection
                    peduncle_count += 1
                                        
                    # These are in RealSense coordinate system
                    self.peduncle_center.x = poi_y
                    self.peduncle_center.y = poi_x

                    self.peduncle_center.z = 0 # self.get_depth(int(self.peduncle_center.x), int(self.peduncle_center.y))
                    
                    # RS frame
                    X, Y, Z = 0, 0, 0 #self.get_3D_coords(
                        # self.peduncle_center.x, self.peduncle_center.y, self.peduncle_center.z)

                    self.peduncle_marker_rs.points.append(Point(X, Y, Z))
                    self.peduncle_marker_rs.header.stamp = rospy.Time.now()
                    self.peduncle_marker_rs_pub.publish(self.peduncle_marker_rs)

                    # Base frame
                    # X_b, Y_b, Z_b = self.transform_to_base_frame(X, Y, Z)

                    # self.peduncle_marker_base.points.append(Point(X_b, Y_b, Z_b))
                    # self.peduncle_marker_base.header.stamp = rospy.Time.now()
                    # self.peduncle_marker_base_pub.publish(self.peduncle_marker_base)

                    self.box_size = (box[2] - box[0]) * (box[3] - box[1])

                    self.last_peduncle_center = self.peduncle_center
            
        else:
            self.detection_void_count += 1
            print("no detection!!")
            if self.detection_void_count > 100:
                self.peduncle_center.x = self.last_peduncle_center.x
                self.peduncle_center.y = 240
                self.pepper_center.x = self.last_pepper_center.x
                self.pepper_center.y = 240
                self.detection_void_count = 0

        self.poi_pub.publish(self.poi)
        self.pepper_center_pub.publish(self.pepper_center)
        self.peduncle_center_pub.publish(self.peduncle_center)

        return image

    def depth_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("Error converting from depth image message: {}".format(e))
            
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
        # Get the 3D coordinates of the pixel in the RS frame? TODO
        Z = z
        X = (x - self.camera_matrix[0, 2]) * Z / self.camera_matrix[0, 0]
        Y = (y - self.camera_matrix[1, 2]) * Z / self.camera_matrix[1, 1]
        return X, Y, Z
    
    def transform_to_base_frame(self, X, Y, Z):
        # Get transform
        try:
            transformation = self.tfBuffer.lookup_transform("link_base", "camera_color_optical_frame", rospy.Time(), rospy.Duration(0.1))
            
            # Get translation and rotation
            trans, quat = transformation.transform.translation, transformation.transform.rotation

            # Create homogeneous matrix
            homo_matrix = np.asarray(quaternion_matrix([quat.x, quat.y, quat.z, quat.w]))
            homo_matrix[:3, 3] = np.array([trans.x, trans.y, trans.z])

            # Transform to base frame
            point_camera_frame = np.array([X, Y, Z, 1])
            point_base_frame = np.matmul(homo_matrix, point_camera_frame) 

            return point_base_frame[0], point_base_frame[1], point_base_frame[2]
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error getting the transform")

    def state_callback(self, msg):
        self.state = msg.data

    def visualize_result(self, image, segment, color=(100, 0, 125, 0.1)):
        mask_coords = (segment @ np.array([[self.img_width, 0], [0, self.img_height]])).astype(int)
        image = cv2.fillPoly(image, pts=[mask_coords], color=color)
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
    
    def random_color(self):
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        a = random.random()  # returns a float between 0 and 1
        return (r, g, b, a)

if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass