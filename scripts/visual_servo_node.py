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
            rel_z=0.0  
            if self.joy_state.axes[2] <= 1.0:
                rel_z = -abs(1-self.joy_state.axes[2] )*scale        
                waypoints = []

                wpose = self.arm_group.get_current_pose().pose
            

                print(str(rel_z))
                # import pdb 
                # pdb.set_trace()
                wpose.position.z += rel_z  # First move up (z)
                #wpose.position.y += scale * 0.2  # and sideways (y)
                waypoints.append(copy.deepcopy(wpose))

                #wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
                #waypoints.append(copy.deepcopy(wpose))

                # wpose.position.y -= scale * 0.1  # Third move sideways (y)
                # waypoints.append(copy.deepcopy(wpose))

                # We want the Cartesian path to be interpolated at a resolution of 1 cm
                # which is why we will specify 0.01 as the eef_step in Cartesian
                # translation.  We will disable the jump threshold by setting it to 0.0,
                # ignoring the check for infeasible jumps in joint space, which is sufficient
                # for this tutorial.
                (plan, fraction) = self.arm_group.compute_cartesian_path(
                    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
                )  # jump_threshold

                # Note: We are just planning, not asking move_group to actually move the robot yet:
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

