#!/usr/bin/env python3

import rospy
import math

from xarm.wrapper import XArmAPI
from xarm.version import __version__

"""
Move the xArm using API calls 
"""

class Manipulator:

    def __init__(self):
        # initialize values
        rospy.init_node('manipulation_node', anonymous=True)
        self.ip = rospy.get_param('/robot_ip')
        self.init_pose = rospy.get_param('/init_pose')
        self.basket_pose = rospy.get_param('/basket_pose')

        # initialize xArm
        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.connect()
        self.arm.set_mode(0)
        self.arm.set_state(0)


    def moveToInit(self):
        """move to initial position"""
        self.arm.set_position(self.init_pose[0], self.init_pose[1], self.init_pose[2], self.init_pose[3], self.init_pose[4], self.init_pose[5], self.init_pose[6], self.init_pose[7], wait=True, speed=10)


    def moveToBasket(self):
        """move to basket"""
        self.arm.set_position(self.basket_pose[0], self.basket_pose[1], self.basket_pose[2], self.basket_pose[3], self.basket_pose[4], self.basket_pose[5], self.basket_pose[6], self.basket_pose[7], wait=True, speed=10)    


    def cartesianMove(self,dist):
        """cartesian move along positive x"""
        # convert m to mm
        dist = dist*1000
        x,y,z,roll,pitch,yaw = self.arm.get_position()
        self.arm.set_position(x+dist, y, z, roll, pitch, yaw, wait=True, speed=10) # todo: test out relative=True


    def test(self):
        self.cartesianMove(0.1)
        

if __name__ == '__main__':

    try:
        xarm = Manipulator()
        while not rospy.is_shutdown():
            xarm.test()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
