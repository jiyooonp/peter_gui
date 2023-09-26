
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