#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Global variables
output_file = '~/catkin_ws/src/bag/output_video.mp4'
frame_rate = 30  # Adjust this based on your requirements

# Initialize the ROS node
rospy.init_node('image_to_video_node', anonymous=True)

# Initialize CvBridge
bridge = CvBridge()

# Define a callback function to process incoming images


def image_callback(data):
    print("converting video")
    try:
        # Convert the ROS Image message to a OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # Write the image to the video file
        out.write(cv_image)

    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))


# Subscribe to the ROS image topic
# Change this to your actual image topic
image_topic = '/camera/color/image_raw'
rospy.Subscriber(image_topic, Image, image_callback)

# Initialize the video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# Adjust the resolution as needed
out = cv2.VideoWriter(output_file, fourcc, frame_rate, (640, 480))

# Main loop (spin) to keep the node alive
rospy.spin()

# Release the video writer when finished
print("releasing video")
out.release()
