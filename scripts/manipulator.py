#!/usr/bin/env python3

import rospy
from xarm.wrapper import XArmAPI
from xarm.version import __version__

"""
Move the xArm using API calls 
"""

class Manipulator:

    def __init__(self):
        # initialize values
        self.ip = rospy.get_param('/xarm/xarm_robot_ip')
        # self.ip = '192.168.1.214'
        # self.init_pose = rospy.get_param('/init_pose')
        # self.basket_pose = rospy.get_param('/basket_pose')
        # todo: put these in launch file
        self.init_pose = [262.789368, 0.513268, 505.799835, 87.280666, -44.962863, 84.593953]
        self.basket_pose = [201.56279, -168.17691, 513.328613, 85.901671, -44.935476, 45.608758]

        # initialize xArm
        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.connect()
        self.arm.set_mode(0)
        self.arm.set_state(0)

    def moveToInit(self):
        """move to initial position"""
        print("Moving to initial pose")
        self.arm.set_position(self.init_pose[0], self.init_pose[1], self.init_pose[2], self.init_pose[3], self.init_pose[4], self.init_pose[5], wait=True)

    def moveToBasket(self, dist):
        """move to basket"""
        self.cartesianMove(-dist) # move backwards so we don't hit the plant
        print("Moving to basket")
        self.arm.set_position(self.basket_pose[0], self.basket_pose[1], self.basket_pose[2], self.basket_pose[3], self.basket_pose[4], self.basket_pose[5], wait=True)    
        # todo: need to test this approach: don't know if this is better or if it's better to only move the first joint

    def cartesianMove(self,dist):
        """cartesian move along x"""
        dist = dist*1000  # convert m to mm
        x,y,z,roll,pitch,yaw = self.arm.get_position()[1]
        print("Executing cartesian move")
        self.arm.set_position(x+dist, y, z, roll, pitch, yaw, wait=True) # todo: test out relative=True

    # def test(self):
    #     print("TESTING")
    #     self.moveToInit()
    #     # self.moveToBasket(0.05)
    #     return


if __name__ == '__main__':

    try:
        xarm = Manipulator()
        while not rospy.is_shutdown():
            # xarm.test()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
