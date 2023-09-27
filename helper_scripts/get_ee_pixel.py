#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

# Global variables to store the image and the pixel picked
image = None
picked_pixel = None

def image_callback(msg):
    global image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    

def main():
    rospy.init_node("pixel_picker")
    image_topic = "/camera/color/image_raw"  # Change this to the actual image topic name
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.sleep(5)
    
    plt.imshow(image)
    plt.show()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
