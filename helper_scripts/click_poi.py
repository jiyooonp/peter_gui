#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

class ImageClickListener:
    def __init__(self):
        self.bridge = CvBridge()
        
        # ROS subscriber and publisher
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.point_pub = rospy.Publisher("/clicked_poi", Point, queue_size=10)
        
        self.window_name = "Click on POI"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)
        
    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            point = Point()
            point.x = x
            point.y = y
            self.point_pub.publish(point)

if __name__ == '__main__':
    rospy.init_node('image_click_listener', anonymous=True)
    icl = ImageClickListener()
    rospy.spin()
    cv2.destroyAllWindows()
