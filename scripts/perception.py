#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node')
        self.bridge = CvBridge()

        # Define the RealSense image subscriber
        self.image_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Define the YOLO model
        self.yolo_pepper = YOLO('/home/robotics/ISU_Demo/src/ISU_Demo/scripts/yolov5s.pt')
        self.yolo_peduncle = YOLO('/home/robotics/ISU_Demo/src/ISU_Demo/scripts/yolov5s.pt')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr("Error converting Image message: {}".format(e))
            return

        # Run YOLO on the image
        detected_image = self.run_yolo(cv_image)

        # Process the detected_image as desired (e.g., publish or display)

    def run_yolo(self, image):
        results_pepper = self.yolo_pepper(image)
        results_peduncle = self.yolo_peduncle(image)
        
        #check if there is a bounding box
        if results_pepper.pred[0] is not None:
            # get the bounding box of the first detected pepper
            pepper_bbox = results_pepper.pred[0][0]
           
        #check if there is a bounding box for penducle
        if results_peduncle.pred[0] is not None:
            peduncle_bbox = results_peduncle.pred[0][0]
         


        return image

if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
