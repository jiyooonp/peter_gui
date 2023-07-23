#!/usr/bin/env python3
import rospy
import cv2
import copy
import numpy as np
import sys
import moveit_commander
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class VisualServoingNode:
    def __init__(self):

        rospy.init_node('visual_servoing_node', anonymous=True)
        self.rate= rospy.Rate(30)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.joystick_callback = rospy.Subscriber('/joy', Joy, self.joystick_callback,queue_size=1)

        #self.joystick_callback = rospy.Subscriber('/keyboard', Int16, self.joystick_callback)
        self.target_x = rospy.get_param('~target_x', 320)  # Target x-coordinate in the image center
        self.target_y = rospy.get_param('~target_y', 240)  # Target y-coordinate in the image center
        self.k_p = rospy.get_param('~p_gain', 0.001)  # Proportional gain for visual servoing
        self.joy_state = Joy()
        moveit_commander.roscpp_initialize([])
        self.arm_group =moveit_commander.MoveGroupCommander("xarm6")

        # # Set the planning reference frame (usually the base_link)
        # self.arm_group.set_pose_reference_frame("link_base")

        # # Set the end effector link (usually the last link in the robot's arm)
        # self.arm_group.set_end_effector_link("link_eef")


    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # Perform image processing or feature extraction (you can use OpenCV or other libraries)

        # Calculate error between target and actual feature position
        target_error_x = self.target_x - target_actual_x
        target_error_y = self.target_y - target_actual_y

        # Visual Servoing control law
        vel_cmd = Twist()
        vel_cmd.linear.x = self.k_p * target_error_y
        vel_cmd.angular.z = self.k_p * target_error_x

        self.cmd_vel_pub.publish(vel_cmd)


    def joystick_callback(self, data):
        self.joy_state = data
        
    def teleop(self):
        # process incoming joystick message
        if self.joy_state!=Joy():
            scale = 0.001
            rel_x=0.0
            rel_y=0.0
            rel_z=0.0

            # start with the current pose
            waypoints = []
            wpose = self.arm_group.get_current_pose().pose

            # check if negative z axis is pressed
            if self.joy_state.axes[2] <= 1.0:
                rel_z = -abs(1-self.joy_state.axes[2] )*scale      
            
            # check if positive z axis is pressed
            if self.joy_state.axes[5] <= 1.0:
                rel_z = abs(1-self.joy_state.axes[5] )*scale
   
            # check if x axis is pressed 
            if self.joy_state.axes[3] != 0.0:
                rel_x = self.joy_state.axes[3]*scale

            # check if y axis is pressed
            if self.joy_state.axes[4] != 0.0:
                rel_y = self.joy_state.axes[4]*scale
            
            # only plan if there is a change in the pose
            if rel_x == 0.0 and rel_y == 0.0 and rel_z == 0.0:
                return

            else:
                # add all relative values to the current pose
                wpose.position.x += rel_x
                wpose.position.y += rel_y  
                wpose.position.z += rel_z 
            
                waypoints.append(copy.deepcopy(wpose))

              
                (plan, fraction) = self.arm_group.compute_cartesian_path(
                    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
                )  # jump_threshold
                
                self.move(plan)
        

    def move(self, plan):
        self.arm_group.execute(plan, wait=True)

    def run(self):
        while not rospy.is_shutdown():
            self.teleop()
            self.rate.sleep()


    
if __name__ == '__main__':
    try:
        visual_servoing_node = VisualServoingNode()        
        visual_servoing_node.run()
    except rospy.ROSInterruptException:
        pass

