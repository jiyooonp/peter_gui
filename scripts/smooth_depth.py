#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class DepthImageInterpolator:
    def __init__(self):
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        # self.pub_5 = rospy.Publisher("/camera/interpolated_depth_5", Image, queue_size=1)
        self.pub_2 = rospy.Publisher("/camera/interpolated_depth_2", Image, queue_size=1)
        # self.pub_10 = rospy.Publisher("/camera/interpolated_depth_10", Image, queue_size=1)

    def depth_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        # Perform interpolation on cv_image here
        # One basic method is to use OpenCV's inpainting function
        mask = (cv_image == 0).astype(np.uint8)
        # interpolated_image_5 = cv2.inpaint(cv_image, mask, inpaintRadius=5, flags=cv2.INPAINT_TELEA)
        interpolated_image_2 = cv2.inpaint(cv_image, mask, inpaintRadius=2, flags=cv2.INPAINT_TELEA)
        # interpolated_image_10 = cv2.inpaint(cv_image, mask, inpaintRadius=10, flags=cv2.INPAINT_TELEA)

        # Convert the interpolated image back to a ROS Image message
        try:
            # ros_image_5 = self.bridge.cv2_to_imgmsg(interpolated_image_5, encoding="passthrough")
            ros_image_2 = self.bridge.cv2_to_imgmsg(interpolated_image_2, encoding="passthrough")
            # ros_image_10 = self.bridge.cv2_to_imgmsg(interpolated_image_10, encoding="passthrough")

        except CvBridgeError as e:
            print(e)

        # Publish the interpolated image
        # self.pub_5.publish(ros_image_5)
        self.pub_2.publish(ros_image_2)
        # self.pub_10.publish(ros_image_10)

if __name__ == '__main__':
    rospy.init_node('depth_image_interpolator', anonymous=True)
    interpolator = DepthImageInterpolator()
    rospy.spin()
