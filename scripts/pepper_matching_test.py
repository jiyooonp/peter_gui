#!/usr/bin/env python3

import rospy
import cv_bridge
import rospkg

import cv2
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image as PILImage

from ultralytics import YOLO

import os
import io
import random

from pepper_util import PepperPeduncle, PepperFruit, Pepper
from match_peppers_util import match_pepper_fruit_peduncle

import matplotlib.pyplot as plt

rospy.init_node('perception_node', anonymous=True)
img_width = 640
img_height = 480

rospack = rospkg.RosPack()
package_name = 'fvd_ws'
package_path = rospack.get_path(package_name)

# Define the YOLO model
yolo = YOLO(
    package_path+'/weights/red.pt')

# store the results of YOLO
fruit_count = 0
peduncle_count = 0
pepper_count = 0
peduncle_detections = dict()
pepper_detections = dict()
fruit_detections = dict()


# visualization
image_count = 0

def visualize_result(image, segment, poi=None, color=(100, 0, 125, 0.1)):
        mask_coords = (segment @ np.array([[img_width, 0], [0, img_height]])).astype(int)
        image = cv2.fillPoly(image, pts=[mask_coords], color=color)
        if poi is not None:
            image = cv2.circle(image, (int(poi[1]), int(poi[0])), 5, (0, 0, 255), -1)
        return image

def run_yolo(image):
    global fruit_count, peduncle_count, image_count, pepper_count, peduncle_detections, fruit_detections
    results_both = yolo(image, verbose=False)

    result = results_both[0] # only take the first image because there is only one image
    if len(result.boxes) != 0: # if there is a detection

        for i in range(len(result.masks)): # for each mask
            segment = result.masks.xyn[i] # only boundary of mask
            mask = result.masks.data[i]       # mask with 1s and 0s
            box = result.boxes.xyxy[i]   
            cls = result.boxes.cls[i] # 0 is pepper, 1 is peduncle


            xywh = result.boxes.xywh[i].cpu().numpy() # TODO why is this 0 not i?
            # Switch from YOLO axes to NumPy axes
            xywh[0], xywh[1] = xywh[1], xywh[0]

            if cls == 1: # it is a pepper
                pepper_detection = PepperFruit(fruit_count, segment=segment)

                pepper_detection.xywh = xywh
                fruit_detections[fruit_count] = pepper_detection
                fruit_count+= 1

            else: # it is a peduncle
                
                peduncle_detection = PepperPeduncle(peduncle_count, np.array(mask.cpu()), segment=segment)

                peduncle_detection.xywh = xywh
                peduncle_detections[peduncle_count] = peduncle_detection
                peduncle_count += 1

            rand_color = random_color()
            for p in peduncle_detections.values():
                temp_image = visualize_result(image, p.segment, poi=None, color=rand_color)
            for f in fruit_detections.values():
                temp_image = visualize_result(image, f.segment, poi=None, color=rand_color) 

            plt.imsave(package_path+'/viz/result_orig_'+str(image_count)+'.png', temp_image)    
             
        image_count+=1


def random_color():
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    a = random.random()  # returns a float between 0 and 1
    return (r, g, b, a)


for i in range(5, 7):
        
    cv_image = cv2.imread(package_path+'/viz/'+str(i)+'.png')
    # resize image to 640, 480
    cv_image = cv2.resize(cv_image, (640, 480))
    print("Doing image #"+str(i))

    run_yolo(cv_image)
        
    pepper_fruit_peduncle_match = match_pepper_fruit_peduncle(fruit_detections, peduncle_detections, i, cv_image)

    for (pfn, ppn), _ in pepper_fruit_peduncle_match:
        if ppn == -1:
            continue
        else:
            pepper = Pepper(pepper_count, pfn, ppn)
            pepper.pepper_fruit = fruit_detections[pfn]
            pepper.pepper_fruit.parent_pepper = pepper_count
            pepper.pepper_peduncle = peduncle_detections[ppn]
            pepper.pepper_peduncle.parent_pepper = pepper_count
            pepper_detections[pepper_count] = pepper
            pepper_count += 1
    print("pepper_detections:", pepper_detections)
    if pepper_detections != dict():
        for i, pepper in pepper_detections.items():
            rand_color = random_color()
            image = visualize_result(cv_image, pepper.pepper_fruit.segment, poi=None, color=rand_color)
            image = visualize_result(image, pepper.pepper_peduncle.segment, poi=pepper.pepper_peduncle.poi_px, color=rand_color) 
    plt.imsave(package_path+'/viz/result_'+str(image_count)+'.png', image)   

    fruit_detections = dict()
    peduncle_detections = dict()

    image = cv_image

        
    pepper_detections = dict()

