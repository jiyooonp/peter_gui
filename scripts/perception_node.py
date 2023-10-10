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
import tf2_ros
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String, Int16
from tf.transformations import quaternion_matrix
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
        self.peduncle_marker_rs = Marker()
        self.peduncle_marker_rs.type = 8
        self.peduncle_marker_rs.header.frame_id = "camera_color_optical_frame"
        self.peduncle_marker_rs.color.r = 0.0
        self.peduncle_marker_rs.color.g = 0.0
        self.peduncle_marker_rs.color.b = 1.0
        self.peduncle_marker_rs.color.a = 1.0
        self.peduncle_marker_rs.scale.x = 0.05
        self.peduncle_marker_rs.scale.y = 0.05

        self.peduncle_marker_base = Marker()
        self.peduncle_marker_base.type = 8
        self.peduncle_marker_base.header.frame_id = "link_base"
        self.peduncle_marker_base.color.r = 1.0
        self.peduncle_marker_base.color.g = 0.0
        self.peduncle_marker_base.color.b = 1.0
        self.peduncle_marker_base.color.a = 1.0
        self.peduncle_marker_base.scale.x = 0.02
        self.peduncle_marker_base.scale.y = 0.02

        self.pepper_marker_rs = Marker()
        self.pepper_marker_rs.type = 8
        self.pepper_marker_rs.header.frame_id = "camera_color_optical_frame"
        self.pepper_marker_rs.color.r = 1.0
        self.pepper_marker_rs.color.g = 0.0
        self.pepper_marker_rs.color.b = 0.0
        self.pepper_marker_rs.color.a = 1.0
        self.pepper_marker_rs.scale.x = 0.05
        self.pepper_marker_rs.scale.y = 0.05

        self.pepper_marker_base = Marker()
        self.pepper_marker_base.type = 8
        self.pepper_marker_base.header.frame_id = "link_base"
        self.pepper_marker_base.color.r = 0.0
        self.pepper_marker_base.color.g = 1.0
        self.pepper_marker_base.color.b = 0.0
        self.pepper_marker_base.color.a = 1.0
        self.pepper_marker_base.scale.x = 0.06
        self.pepper_marker_base.scale.y = 0.06

        self.go_straight = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Define the RealSense image subscriber
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        self.peduncle_marker_rs_pub = rospy.Publisher("/visualization_peduncle_marker_rs", Marker, queue_size=1)
        self.peduncle_marker_base_pub = rospy.Publisher("/visualization_peduncle_marker_base", Marker, queue_size=1)

        self.pepper_marker_rs_pub = rospy.Publisher("/visualization_pepper_marker_rs", Marker, queue_size=1)
        self.pepper_marker_base_pub = rospy.Publisher("/visualization_pepper_marker_base", Marker, queue_size=1)

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
        
        self.poi = Point()
        self.poi_pub = rospy.Publisher(
            '/poi', Point, queue_size=1)
        
        self.state = None
        self.state_sub = rospy.Subscriber('/state', Int16, self.state_callback, queue_size=1)
        
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

        self.pepper_marker_rs.points = []
        self.pepper_marker_base.points = []
        self.peduncle_marker_rs.points = []
        self.peduncle_marker_base.points = []

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
                    
                    self.pepper_marker_rs.points.append(Point(X, Y, Z))
                    self.pepper_marker_rs.header.stamp = rospy.Time.now()
                    self.pepper_marker_rs_pub.publish(self.pepper_marker_rs)

                    # X, Y, Z in base frame
                    X_b, Y_b, Z_b = self.transform_to_base_frame(X, Y, Z)

                    self.pepper_marker_base.points.append(Point(X_b, Y_b, Z_b))
                    self.pepper_marker_base.header.stamp = rospy.Time.now()
                    self.pepper_marker_base_pub.publish(self.pepper_marker_base)

                    if self.state != 5:
                        self.poi.x = X_b
                        self.poi.y = Y_b
                        self.poi.z = Z_b

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

                    self.peduncle_marker_rs.points.append(Point(X, Y, Z))
                    self.peduncle_marker_rs.header.stamp = rospy.Time.now()
                    self.peduncle_marker_rs_pub.publish(self.peduncle_marker_rs)

                    # Base frame
                    X_b, Y_b, Z_b = self.transform_to_base_frame(X, Y, Z)

                    self.peduncle_marker_base.points.append(Point(X_b, Y_b, Z_b))
                    self.peduncle_marker_base.header.stamp = rospy.Time.now()
                    self.peduncle_marker_base_pub.publish(self.peduncle_marker_base)

                    self.box_size = (box[2] - box[0]) * (box[3] - box[1])

                    self.last_peduncle_center = self.peduncle_center

                    if self.box_size > 5000:
                        self.go_straight = True
        else:
            self.detection_void_count += 1
            print("no detection!!")
            if self.detection_void_count > 100:
                self.peduncle_center.x = self.last_peduncle_center.x
                self.peduncle_center.y = 240
                self.pepper_center.x = self.last_pepper_center.x
                self.pepper_center.y = 240
                self.detection_void_count = 0
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\nn\n\n\n\nn\n\n\n\n\nn\n\n\n\nn\n\nn\n\n\nn\n")

        self.poi_pub.publish(self.poi)
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


if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




# names: {0: 'bell-pepper-green', 1: 'bell-pepper-peduncle'}
