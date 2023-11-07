#!/usr/bin/env python3

import rospy
import rospkg
import cv_bridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int16, Bool
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker

import message_filters

import cv2
import torch
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image as PILImage

import message_filters

import tf2_ros
from tf.transformations import quaternion_matrix

from ultralytics import YOLO

import os
import io
import random

from pepper_util import PepperPeduncle, PepperFruit, Pepper
from match_peppers_util import match_pepper_fruit_peduncle

import matplotlib.pyplot as plt


def plot_3d_points(points1, points2, img, filename):
    # Create a new figure
    fig = plt.figure(figsize=(10, 5))  # Adjust the figure size as needed

    # Add subplot for the image
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.imshow(img)
    ax1.axis('off')  # Turn off axis numbers and ticks

    # Add subplot for 3D scatter plot
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    
    # Unpack the first set of points into x1, y1, and z1 lists
    x1, y1, z1 = zip(*points1)
    ax2.scatter(z1, x1, y1, color='blue', label='pepper', alpha=0.2, s = 1)
    
    # Unpack the second set of points into x2, y2, and z2 lists
    x2, y2, z2 = zip(*points2)
    ax2.scatter(z2, x2, y2, color='red', label='peduncle', alpha=0.2, s = 1)
    
    ax2.set_xlabel('depth (m)')
    ax2.set_ylabel('x (m)')
    ax2.set_zlabel('y (m)')

    # Set fixed axis limits and invert the necessary axes
    ax2.set_xlim(5, 0)  # x-axis (depth) from 5 to 0 meters (reversed)
    ax2.set_ylim(-1, 1)  # y-axis from -1 to 1
    ax2.set_zlim(1, 0)  # z-axis from 1 to 0 (reversed)
    
    # Add a legend to differentiate the two point sets
    ax2.legend()

    # Adjust layout
    plt.tight_layout()

    # Save the combined plot to a file
    plt.savefig(filename)

    # Close the plot to free up memory
    plt.close(fig)


    
def make_marker(marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.05, y=0.05):
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

def random_color():
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    a = random.random()  # returns a float between 0 and 1
    return (r, g, b, a)