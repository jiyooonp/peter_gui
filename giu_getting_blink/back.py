#!/usr/bin/env python3
from flask import Flask, render_template, Response
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import base64
from std_msgs.msg import Int16, Bool
from flask_socketio import SocketIO, emit
import numpy as np
app = Flask(__name__)

socketio = SocketIO(app)
bridge = CvBridge()
image_data = None
lock = threading.Lock()
state = -1

def image_callback(msg):
    global image_data
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        ret, jpeg = cv2.imencode('.jpg', cv_image)
        image_data = jpeg.tobytes()
    except Exception as e:
        rospy.logerr("Could not convert image: %s" % e)

def state_callback(msg):
    global state
    state = msg.data
    # Emit the state to all connected clients
    # print("state:", state)
    socketio.emit('state_update', {'state': state})

def ros_thread():
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.Subscriber('/state', Int16, state_callback)
    rospy.spin()

@app.route('/')
def index():
    return render_template('index.html')
@app.route('/stream')
def stream():
    return render_template('front.html')

def generate():
    global image_data
    while not rospy.is_shutdown():
        if image_data is not None:
            frame = image_data
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # If no image, showing pink image
            pink_image = np.zeros((500, 500, 3), dtype=np.uint8)
            pink_image[:] = (255, 105, 180)  # RGB for pink
            ret, jpeg = cv2.imencode('.jpg', pink_image)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    rospy.init_node('image_listener', anonymous=True, disable_signals=True)
    t = threading.Thread(target=ros_thread)
    t.daemon = True
    t.start()
    # app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
