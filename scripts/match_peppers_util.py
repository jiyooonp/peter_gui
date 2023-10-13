from typing import List, Optional, Dict
import math 
import collections
from shapely import Polygon
from pepper_util import PepperPeduncle, PepperFruit

def distance_between_pepper_fruit_peduncle(pepper_fruit: PepperFruit, pepper_peduncle: PepperPeduncle):
    pepper_fruit_xywh = pepper_fruit.xywh
    pepper_peduncle_xywh = pepper_peduncle.xywh

    pf_coords = [pepper_fruit_xywh[0].item(), pepper_fruit_xywh[1].item()]
    pp_coords = [pepper_peduncle_xywh[0].item(), pepper_peduncle_xywh[1].item()]
    distance = math.dist(pf_coords, pp_coords)
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
                                pepper_peduncle_detections: Dict[int, PepperPeduncle]):
    
    pepper_fruit_peduncle_distances = []

    for pepper_fruit in pepper_fruit_detections.values():

        min_dist = math.inf
        peduncle_match = None

        for pepper_peduncle in pepper_peduncle_detections.values():

            dist = distance_between_pepper_fruit_peduncle(pepper_fruit, pepper_peduncle)

            x, y, w, h = pepper_fruit.xywh
            box1 = [[x - w / 2, y - h / 2], [x + w / 2, y - h / 2], [x + w / 2, y + h / 2], [x - w / 2, y + h / 2]]
            x, y, w, h = pepper_peduncle.xywh
            box2 = [[x - w / 2, y - h / 2], [x + w / 2, y - h / 2], [x + w / 2, y + h / 2], [x - w / 2, y + h / 2]]

            iou = calculate_iou(box1, box2)

            if dist < min_dist and iou > 0:
                peduncle_match = pepper_peduncle
                min_dist = dist

        if not peduncle_match:
            peduncle_match = PepperPeduncle(-1)

        pepper_fruit_peduncle_distances.append(((pepper_fruit.number, peduncle_match.number), min_dist))

    pepper_fruit_peduncle_match = remove_duplicate_peduncles(pepper_fruit_peduncle_distances)
    return pepper_fruit_peduncle_match