from typing import List, Optional, Dict
import math 
import collections
from shapely import Polygon
from pepper_util import PepperPeduncle, PepperFruit

import matplotlib.pyplot as plt
import matplotlib.patches as patches

import cv2
def visualize_distance(pepper_fruit, pepper_peduncle, image_count, image):
    if pepper_peduncle.number == -1:
        return

    # axis 
    fxywh = [pepper_fruit.xywh[1], pepper_fruit.xywh[0], pepper_fruit.xywh[3], pepper_fruit.xywh[2]]
    pxywh = [pepper_peduncle.xywh[1], pepper_peduncle.xywh[0], pepper_peduncle.xywh[3], pepper_peduncle.xywh[2]]

    # Convert OpenCV image (BGR) to RGB for matplotlib
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.imshow(image_rgb)  # Display the image

    # Compute top-left coordinates for pepper fruit
    fruit_x = fxywh[0] - fxywh[2]/2
    fruit_y = fxywh[1] - fxywh[3]/2

    # Draw pepper fruit rectangle
    fruit_rect = patches.Rectangle((fruit_x, fruit_y), 
                                   fxywh[2], fxywh[3], 
                                   linewidth=1, edgecolor='r', facecolor='none', label='Pepper Fruit')
    ax.add_patch(fruit_rect)

    # Compute top-left coordinates for pepper peduncle
    peduncle_x = pxywh[0] - pxywh[2]/2
    peduncle_y = pxywh[1] - pxywh[3]/2
    
    # Draw pepper peduncle rectangle
    peduncle_rect = patches.Rectangle((peduncle_x, peduncle_y), 
                                      pxywh[2], pxywh[3], 
                                      linewidth=1, edgecolor='b', facecolor='none', label='Pepper Peduncle')
    ax.add_patch(peduncle_rect)
    
    # Draw a line between the centers of the two rectangles
    fruit_center = (fxywh[0], fxywh[1])
    peduncle_center = (pxywh[0], pxywh[1])
    ax.plot([fruit_center[0], peduncle_center[0]], [fruit_center[1], peduncle_center[1]], 'g--')

    # Hide axes (if desired)
    ax.axis('on') # Use 'on' to display axes
    
    # Save the image
    output_path = '/root/catkin_ws/src/fvd_ws/yeah/pepper_fruit_peduncle_distance'+str(image_count)+'.png'
    plt.savefig(output_path, bbox_inches='tight', pad_inches=0)
    plt.close(fig)

def distance_between_pepper_fruit_peduncle(pepper_fruit: PepperFruit, pepper_peduncle: PepperPeduncle, image_count, image):
    
    pepper_fruit_xywh = pepper_fruit.xywh
    pepper_peduncle_xywh = pepper_peduncle.xywh

    pf_coords = [pepper_fruit_xywh[0].item(), pepper_fruit_xywh[1].item()]

    pp_coords = [pepper_peduncle_xywh[0].item(), pepper_peduncle_xywh[1].item()]

    distance = math.dist(pf_coords, pp_coords)

    # visualize_distance(pepper_fruit, pepper_peduncle, image_count, image)
    return distance

def calculate_iou(box_1, box_2):
    '''
    :param box_1: (4 x 2) tl, tr, br, bl
    :param box_2: (4 x 2) tl, tr, br, bl
    :return: iou from 0-1
    '''
    poly_1 = Polygon(box_1)
    poly_2 = Polygon(box_2)

    if poly_1.intersects(poly_2): 
        iou = poly_1.intersection(poly_2).area / poly_1.union(poly_2).area
    else:
        iou = 0

    # print(f"iou: {iou}")
    return iou

def remove_duplicate_peduncles(pepper_fruit_peduncle_distances: list): # TODO: look into it and wtf am i doing?
    # remove duplicate peduncles

    detetected_pepper_fruit = []
    detected_pepper_peduncle = []

    for (pf, pp), d in pepper_fruit_peduncle_distances:
        detetected_pepper_fruit.append(pf)
        detected_pepper_peduncle.append(pp)
    duplicate_pepper_fruit = [item for item, count in collections.Counter(detetected_pepper_fruit).items() if count > 1]
    duplicate_pepper_peduncle = [item for item, count in collections.Counter(detected_pepper_peduncle).items() if
                                 count > 1]

    pp_duplicate_list = list()
    for pepper_peduncle in duplicate_pepper_peduncle:  # index of peppers
        duplicate_list = list()
        for i in range(len(pepper_fruit_peduncle_distances)):
            (pf, pp), d = pepper_fruit_peduncle_distances[i]
            if pp == pepper_peduncle:
                duplicate_list.append(pepper_fruit_peduncle_distances[i])
        pp_duplicate_list.append(duplicate_list)
    pepper_peduncle_delete = choose_unmatching(pp_duplicate_list)
    for d in pepper_fruit_peduncle_distances:
        if d in pepper_peduncle_delete:
            pepper_fruit_peduncle_distances.remove(d)

    return pepper_fruit_peduncle_distances

def choose_unmatching(duplicate_list):
    # ((pf_number, pp_number), distance)
    pepper_delete = list()
    for duplicates in duplicate_list:
        duplicates = sorted(duplicates, key=lambda d: d[1])
        pepper_delete.append(duplicates[1:])
    return pepper_delete

def match_pepper_fruit_peduncle(pepper_fruit_detections: Dict[int, PepperFruit],
                                pepper_peduncle_detections: Dict[int, PepperPeduncle], image_count: int, image):
    # import ipdb; ipdb.set_trace()
    pepper_fruit_peduncle_distances = []
    pepper_number = 1
    for i, pepper_fruit in pepper_fruit_detections.items():

        min_dist = math.inf
        peduncle_match = None
        peduncle_number = 1
        for pepper_peduncle in pepper_peduncle_detections.values():

            dist = distance_between_pepper_fruit_peduncle(pepper_fruit, pepper_peduncle, image_count*100 + pepper_number*10 + peduncle_number, image)
            x, y, w, h = pepper_fruit.xywh
            box1 = [[x - h / 2, y - w / 2], [x + h / 2, y - w / 2], [x + h / 2, y + w / 2], [x - h / 2, y + w / 2]]

            x, y, w, h = pepper_peduncle.xywh
            box2 = [[x - h / 2, y - w / 2], [x + h / 2, y - w / 2], [x + h / 2, y + w / 2], [x - h / 2, y + w / 2]]

            iou = calculate_iou(box1, box2)
            # print(f"IOU of pepper {pepper_number}, peduncle {peduncle_number}: {iou}")
            if dist < min_dist and iou > 0:
                peduncle_match = pepper_peduncle
                min_dist = dist
            peduncle_number += 1
            
        pepper_number += 1
        if not peduncle_match:
            peduncle_match = PepperPeduncle(-1)

        pepper_fruit_peduncle_distances.append(((pepper_fruit.number, peduncle_match.number), min_dist))
        # visualize_distance(pepper_fruit, peduncle_match, image_count*100 + pepper_number*10 + peduncle_number, image)
# 
    pepper_fruit_peduncle_match = pepper_fruit_peduncle_distances #remove_duplicate_peduncles(pepper_fruit_peduncle_distances)
    for i, ((fruit, peduncle), _ )in enumerate(pepper_fruit_peduncle_distances):
        if peduncle == -1:
            continue
        # visualize_distance(pepper_fruit_detections[fruit], pepper_peduncle_detections[peduncle], image_count*10000+i, image)
    return pepper_fruit_peduncle_match