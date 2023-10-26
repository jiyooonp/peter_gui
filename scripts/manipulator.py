#!/usr/bin/env python3

import rospy
from xarm.wrapper import XArmAPI
from xarm.version import __version__
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


"""
Move the xArm using API calls 
"""

class Manipulator:

    def __init__(self):
        # initialize values
        # self.ip = rospy.get_param('xarm_robot_ip')
        self.ip = "192.168.1.196"
        self.pregrasp_offset = 0.15
        self.ee_length_offset = 0.15
        # todo: put these in config file
        # self.init_pose = [200, 0, 500, 87.280666, -44.962863, 84.593953]# ip: 14
        self.init_pose = [39.866951, -234.744492, 559.317017, 95.052323, -41.443011, 6.623965]
        # self.init_pose = [156, 0.0, 475, -90, 45, -90] # ip:13, upside-down
        self.basket_pose = [201.56279, -168.17691, 513.328613, -90, 45, -90]# ip: 13
        self.basket_pregrasp = [-226.275833, 5.253029, 439.106873, 87.818043, -41.550498, -90.103916]

        # initialize xArm
        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.connect()
        self.arm.set_mode(0)
        self.arm.set_state(0)

    def moveToInit(self):
        """move to initial position"""
        print("Moving to initial pose")
        self.arm.set_position(self.init_pose[0], self.init_pose[1], self.init_pose[2], self.init_pose[3], self.init_pose[4], self.init_pose[5], wait=True, speed=20)

    def moveToBasket(self, dist):
        """move to basket"""
        self.cartesianMove(-dist) # move backwards so we don't hit the plant
        print("Moving to basket")
        self.arm.set_position(self.basket_pose[0], self.basket_pose[1], self.basket_pose[2], self.basket_pose[3], self.basket_pose[4], self.basket_pose[5], wait=True)    

    def cartesianMove(self,dist):
        """cartesian move along x"""
        dist = dist*1000  # convert m to mm
        x,y,z,roll,pitch,yaw = self.arm.get_position()[1]
        print("Executing cartesian move")
        self.arm.set_position(x+dist, y, z, roll, pitch, yaw, wait=True, speed=10) # todo: test out relative=True

    def moveToPregrasp(self,x,y,z):
        x -= self.pregrasp_offset # pregrasp offset
        x -= self.ee_length_offset # ee length offset

        # just using the orientation values from the init position for now
        self.arm.set_position(x * 1000 ,y * 1000 ,z * 1000 ,*self.init_pose[3:], wait=True, speed=20)

    def basketTesting(self):
        # current_pos = self.arm.get_position()[1]
        # print(current_pos)
        self.moveToInit() # move to init pose
        self.cartesianMove(-0.05)
        print("moving to pregrasp")
        self.arm.set_position(*self.basket_pregrasp, wait=True, speed=20) # move to basket pre-grasp
        self.cartesianMove(-0.1) # move forward to basket

    def test(self):
        print("TESTING")
        # self.moveToInit()
        self.basketTesting()
        return
    
    def disconnect(self):
        self.arm.disconnect()
        
    def make_marker(self, marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.05, y=0.05):
        marker = Marker()
        marker.type = marker_type
        marker.header.frame_id = frame_id
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.scale.x = x
        marker.scale.y = y

        return marker


if __name__ == '__main__':

    try:
        xarm = Manipulator()
        while not rospy.is_shutdown():
            # xarm.test()
            rospy.sleep(0.1)
            # rospy.logwarn(xarm.arm.get_position())

    except rospy.ROSInterruptException:
        pass
