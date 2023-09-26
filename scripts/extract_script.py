#!/usr/bin/env python3

import os
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rosbag
import cv2


def extract_images_from_bag(bag_file, topic, output_folder):
    # Check if the output folder exists, and create it if it doesn't
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Initialize CvBridge
    bridge = CvBridge()

    # Counter for image filenames
    count = 0

    # Open the rosbag file
    with rosbag.Bag(bag_file, 'r') as bag:
        # Iterate over the messages in the topic
        for topic, msg, t in bag.read_messages(topics=[topic]):
            try:
                # Convert the image message to an OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except CvBridge.CvBridgeError as e:
                print(e)
                continue

            # Construct the output filename
            output_filename = os.path.join(
                output_folder, f"frame_{count:04d}.png")

            # Save the image
            if count % 20 == 0:
                cv2.imwrite(output_filename, cv_image)
                print(f"Saved: {output_filename}")

            count += 1


if __name__ == '__main__':

    # sys.argv[1]
    bag_file = "/root/catkin_ws/bag/iowa_1_2023-09-25-15-22-28.bag"
    topic = "/camera/color/image_raw"
    output_folder = "/root/catkin_ws/video_bag/bag1"

    extract_images_from_bag(bag_file, topic, output_folder)
