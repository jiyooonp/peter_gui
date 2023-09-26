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

def publish_plt_image():
    # Create a ROS publisher for the Image message

    # Create a CV Bridge to convert the NumPy array to a ROS Image message
    bridge = cv_bridge.CvBridge()

    # Create a simple plot (You can replace this with your actual plotting code)
    x = [1, 2, 3, 4, 5]
    y = [10, 8, 6, 4, 2]
    plt.plot(x, y)

    # Convert the Matplotlib image to a NumPy array
    plt_image_np = np.frombuffer(
        plt.gcf().canvas.tostring_rgb(), dtype=np.uint8)
    plt_image_np = plt_image_np.reshape(
        plt.gcf().canvas.get_width_height()[::-1] + (3,))


def draw_all(img, xywh, mask):
    plt.imshow(img)

    # Draw Peduncle
    mask = mask.masks[0]
    polygon = Polygon(mask)
    x, y = polygon.exterior.xy
    plt.fill(x, y, color="blue", alpha=0.5)
    polygon = Polygon(mask)
    plt.plot(*polygon.exterior.xy)

    # draw pepper
    xywh = xywh[0]
    x = int(xywh[0])
    y = int(xywh[1])
    w = int(xywh[2])
    h = int(xywh[3])
    draw_bounding_box(0, x, y, w, h)

    # Save the plot to a BytesIO buffer as a PNG image
    buffer = io.BytesIO()
    plt.savefig(buffer, format='png')
    buffer.seek(0)

    # Convert the buffer to a NumPy array
    buffer_image = np.array(PILImage.open(buffer))

    # Close the plot to free up resources
    plt.close()

    # Convert the NumPy array to a PIL Image
    pil_image = PILImage.fromarray(buffer_image)

    # Return the PIL Image
    return pil_image


def draw_bounding_box(confidence, x, y, w, h, color="blue", fill=False):
    # Get the current reference
    ax = plt.gca()
    # plot the bounding box
    rect = patches.Rectangle((x - w / 2, y - h / 2), w,
                             h, linewidth=2, edgecolor=color, facecolor='none')
    # Add the patch to the Axes
    ax.add_patch(rect)


def draw_bounding_polygon(confidence, mask, img_shape, color="blue", fill=True):
    mask = mask.xy[0]
    polygon = Polygon(mask)
    x, y = polygon.exterior.xy
    if fill:
        plt.fill(x, y, color=color, alpha=0.7)
    polygon = Polygon(mask)
    plt.plot(*polygon.exterior.xy)


class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node')
        self.bridge = CvBridge()
        self.depth_window = 2  # 5 x 5 window (i-2 to i+2)
        self.camera_matrix = None
        self.img_width = None
        self.img_height = None

        curr_path = os.getcwd()
        print(curr_path)

        # Define the YOLO model
        rospack = rospkg.RosPack()
        package_name = 'perception_refactor'
        package_path = rospack.get_path(package_name)
        print(package_path)

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

        self.peduncle_center = [0, 0, 0]

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            # cv2.imshow("Image window", cv_image)
            if cv_image is not None:
                _ = self.run_yolo(cv_image)

        except CvBridgeError as e:
            rospy.logerr("Error converting from image message: {}".format(e))
            return

        # Run YOLO on the image
        # detected_image = self.run_yolo(cv_image)

    def run_yolo(self, image):
        results_both = self.yolo(image)

        self.peduncle_dict = dict()
        peduncle_count = 0
        # pil_image = None
        # xywh = None
        # mask = None

        self.pepper_center = Point()
        self.pepper_center.x = self.img_width/2
        self.pepper_center.y = self.img_height/2
        self.pepper_center.z = 0


        self.peduncle_center = Point()
        self.peduncle_center.x = 425 #self.img_width/2
        self.peduncle_center.y = 135 # self.img_height/2
        self.peduncle_center.z = 0

        if self.go_straight:
            self.peduncle_center.z = 1.5
            print("going in straight")

        self.pepper_marker.points = []

        if self.pepper_marker.points == []:
            self.pepper_marker.points.append(Point(0, 0, 0))

        self.pepper_marker.header.stamp = rospy.Time.now()
        self.pepper_marker_publisher.publish(self.pepper_marker)

        self.peduncle_marker.points = []

        # print(results_both)

        for i, result in enumerate(results_both): # peduncel and pepper
            if result.boxes.boxes.size(0) != 0: # if there is a result?
                # for i in range(result.masks.shape[0]):
                mask = result.masks[0]
                box = result.boxes[0]  # Boxes object for bbox outputs
                box_peduncle = box.xyxy[0]

                if True:
                    peduncle = PepperPeduncle(i, np.array(mask.data[0].cpu()))
                    poi_x, poi_y = peduncle.set_point_of_interaction()
                    self.peduncle_dict[i] = peduncle
                    
                    # Uncomment this for visualizing POI
                    # print(poi_x, poi_y)
                    # image = peduncle.mask
                    # image = cv2.circle(image, (int(poi_y), int(poi_x)), radius=5, color=0, thickness=-1)
                    # cv2.imshow('Image', image)
                    # cv2.waitKey(0)
                    # cv2.destroyAllWindows()

                    box_peduncle = result.boxes.xyxy[0]  # only take the first bb

                    self.peduncle_center = Point()
                    
                    # Alec's ws
                    # convert mask points [0, 1] to image coordinates
                    # print(mask)
                    peduncle.mask = torch.Tensor(mask.segments[0])
                    mask_coords = (peduncle.mask.numpy() @ np.array([[self.img_width, 0], [0, self.img_height]])).astype(int)

                    cv2.fillPoly(image, pts=[mask_coords], color=(0, 0, 255, 0.1))

                    # peduncle.conf = box.conf[i]
                    # peduncle.xywh = box.xywh[i].cpu().numpy()
                    # peduncle.mask = torch.Tensor(mask.segments[i]).to_

                #     # These are in RealSense coordinate system
                #     self.peduncle_center.x = int((box[0] + box[2]) / 2)
                #     self.peduncle_center.y = int((box[1] + box[3]) / 2)
                
                
                    # These are in RealSense coordinate system
                    self.peduncle_center.x = poi_y
                    self.peduncle_center.y = poi_x

                    self.peduncle_box_size_publisher.publish(str(box_peduncle[2] - box_peduncle[0]) + " " + str(box_peduncle[3] - box_peduncle[1]))

                    self.box_size = (box_peduncle[2] - box_peduncle[0]) * (box_peduncle[3] - box_peduncle[1])

                    if self.box_size > 5000:
                        self.go_straight = True
                        
                    # Depth image is a numpy array so switch coordinates
                    # Depth is converted from mm to m
                    # self.peduncle_center.z = 0.001*self.depth_image[self.peduncle_center.y,
                    #                                                 self.peduncle_center.x]
                    # print(self.peduncle_center.x, type(self.peduncle_center.x))
                    self.peduncle_center.z = self.get_depth(int(self.peduncle_center.x), int(self.peduncle_center.y))
                    # print("depth: ", self.peduncle_center.z)
                    
                    X, Y, Z = self.get_3D_coords(
                        self.peduncle_center.x, self.peduncle_center.y, self.peduncle_center.z)

                    self.peduncle_marker.points.append(Point(X, Y, Z))

                    p3 = (int(box_peduncle[0]), int(box_peduncle[1]))
                    p4 = (int(box_peduncle[2]), int(box_peduncle[3]))
                    cv2.rectangle(image, p3, p4, (255, 0, 0), 10)

        if self.peduncle_marker.points == []:
            self.peduncle_marker.points.append(Point(0, 0, 0))
            
            # if boxes.xyxy.cpu().numpy().size != 0:
                
            #     print("result: ", result)

            #     box_peduncle = boxes.xyxy[0]
            #     box = boxes.xyxy[0]  # only take the first bb

            #     self.peduncle_center = Point()

            #     # These are in RealSense coordinate system
            #     self.peduncle_center.x = int((box[0] + box[2]) / 2)
            #     self.peduncle_center.y = int((box[1] + box[3]) / 2)

            #     self.peduncle_box_size_publisher.publish(str(box_peduncle[2] - box_peduncle[0]) + " " + str(box_peduncle[3] - box_peduncle[1]))

            #     self.box_size = (box_peduncle[2] - box_peduncle[0]) * (box_peduncle[3] - box_peduncle[1])

            #     if self.box_size >5000:
            #         self.go_straight = True
            #     # Depth image is a numpy array so switch coordinates
            #     # Depth is converted from mm to m
            #     # self.peduncle_center.z = 0.001*self.depth_image[self.peduncle_center.y,
            #     #                                                 self.peduncle_center.x]
            #     self.peduncle_center.z = self.get_depth(self.peduncle_center.x, self.peduncle_center.y)
            #     # print("depth: ", self.peduncle_center.z)
            #     X, Y, Z = self.get_3D_coords(
            #         self.peduncle_center.x, self.peduncle_center.y, self.peduncle_center.z)

            #     self.peduncle_marker.points.append(Point(X, Y, Z))

            #     p3 = (int(box_peduncle[0]), int(box_peduncle[1]))
            #     p4 = (int(box_peduncle[2]), int(box_peduncle[3]))
            #     cv2.rectangle(image, p3, p4, (255, 0, 0), 10)

        # if self.peduncle_marker.points == []:
        #     self.peduncle_marker.points.append(Point(0, 0, 0))
            
        # self.peduncle_marker.header.stamp = rospy.Time.now()
        # self.peduncle_marker_publisher.publish(self.peduncle_marker)

        # # self.pepper_center_publisher.publish(self.pepper_center)
        # self.peduncle_center_publisher.publish(self.peduncle_center)


        try:
            image_msg_bb = self.bridge.cv2_to_imgmsg(image, "rgb8")
            self.image_publisher.publish(image_msg_bb)
            print("published image")

        except CvBridgeError as e:
            rospy.logerr(
                "Error converting back to image message: {}".format(e))
            return

        # pil_image = draw_all(image, xywh, mask)
        # np_image = np.array(pil_image)

        # # Convert BGR image to RGB (OpenCV uses BGR by default)
        # rgb_image = np_image
        # # Display the image using cv2.imshow()
        # cv2.imshow('Image', rgb_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

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
        # Get the 3D coordinates of the pixel
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



'''''''''
[ERROR] [1695760745.430541]: bad callback: <bound method PerceptionNode.image_callback of <__main__.PerceptionNode object at 0x7f8bd584dd60>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/root/catkin_ws/src/ISU_Demo/scripts/alec.py", line 181, in image_callback
    _ = self.run_yolo(cv_image)
  File "/root/catkin_ws/src/ISU_Demo/scripts/alec.py", line 254, in run_yolo
    mask_coords = (peduncle.mask @ np.array([[self.img_height, 0], [0, self.img_width]])).astype(int)
AttributeError: 'Tensor' object has no attribute 'astype'




'''