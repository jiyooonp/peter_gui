#!/usr/bin/env python3
import rospy
import cv2
import copy
import numpy as np
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class VisualServoingNode:
    def __init__(self):

        rospy.init_node('visual_servoing_node', anonymous=True)
        self.rate= rospy.Rate(30)
        self.bridge = CvBridge()
        # subscribe to perception bbox messages
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        

       
  


    def perception_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

       

        # Calculate error between bbox center and image center
        target_error_x = self.target_x - target_actual_x
        target_error_y = self.target_y - target_actual_y

        # Visual Servoing control law 
        vel_cmd = Twist()
        vel_cmd.linear.x = self.k_p * target_error_y
        vel_cmd.angular.z = self.k_p * target_error_x

        self.cmd_vel_pub.publish(vel_cmd)

   
    def run(self):
        while not rospy.is_shutdown():
 
            self.rate.sleep()


    
if __name__ == '__main__':
    try:
        visual_servoing_node = VisualServoingNode()        
        visual_servoing_node.run()
    except rospy.ROSInterruptException:
        pass

