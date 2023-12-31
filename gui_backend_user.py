#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16, Bool

from cv_bridge import CvBridge
import cv2
import numpy as np

import threading
import base64

from flask import Flask, render_template, Response, request, jsonify
from flask_socketio import SocketIO, emit
import time
from collections import defaultdict


app = Flask(__name__)

socketio = SocketIO(app)
bridge = CvBridge()
lock = threading.Lock()

image_data = None

system_state = -1
amiga_state = -1

harvest_times = defaultdict()
pepper_id = 1

current_values = {'x': -1, 'y': -1}


ros_publisher = rospy.Publisher('/user_selected_points', String, queue_size=10)


def image_callback(msg):
    global image_data
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        ret, jpeg = cv2.imencode('.jpg', cv_image)
        image_data = jpeg.tobytes()
    except Exception as e:
        rospy.logerr("Could not convert image: %s" % e)


timer_start = time.time()
elapsed_time = 0

def system_state_callback(msg):
    global system_state, timer_start, elapsed_time, pepper_id

    previous_state = system_state
    system_state = msg.data

    # Start timer when state changes to 3
    if system_state == 3 and previous_state != 3:
        pepper_id += 1
        timer_start = time.time()

    # Stop timer and calculate elapsed time when state changes to 8
    elif system_state == 8 and previous_state != 8 and timer_start is not None:
        harvest_times[pepper_id] = time.time() - timer_start
    if system_state < 3:
        pass
    else:
        elapsed_time = time.time() - timer_start
    
        # timer_start = None
    socketio.emit('timer_update', {'pepper_id': pepper_id, 'elapsed_time': elapsed_time})
    # print("Sending timer update", elapsed_time)

    socketio.emit('system_state_update', {'state': system_state})

    continuous_publish()


def amiga_state_callback(msg):
    global amiga_state
    amiga_state = msg.data
    socketio.emit('amiga_state_update', {'state': amiga_state})


def ros_thread():
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.Subscriber('/state', Int16, system_state_callback)
    rospy.Subscriber('/amiga_aligned', Int16, amiga_state_callback)
    continuous_publish()
    rospy.spin()


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/user_select')
def user_select():
    return render_template('user_select.html')


@app.route('/video_feed')
def video_feed():
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# Function to continuously publish data
def continuous_publish():
    # while True:
    # Read the latest values
    x, y = current_values['x'], current_values['y']
    # Publish the data to the ROS topic
    ros_publisher.publish(f'{int(x)},{int(y)}')

@app.route('/send_to_ros', methods=['POST'])
def send_to_ros():
    global current_values
    if request.method == 'POST':
        data = request.json
        current_values['x'] = data['x']
        current_values['y'] = data['y']

        return jsonify({'message': 'Data received'})

def generate():
    global image_data
    start = time.time()
    while not rospy.is_shutdown():
        if time.time() - start > 5:# added because port could not handle too much data
            if image_data is not None:
                frame = image_data
                yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                start = time.time()
            else:
                # If no image, showing pink image
                green_image = np.zeros((500, 500,3), dtype=np.uint8)
                green_image[:] = (151, 188, 98)  # RGB for pink
                ret, jpeg = cv2.imencode('.jpg', green_image)
                yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')


if __name__ == '__main__':
    rospy.init_node('peter_gui', anonymous=True, disable_signals=True)
    t = threading.Thread(target=ros_thread)
    t.daemon = True
    t.start()
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
