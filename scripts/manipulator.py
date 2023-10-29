#!/usr/bin/env python3
import os
import yaml
import rospy
import rospkg
from xarm.wrapper import XArmAPI
from xarm.version import __version__
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


"""
Move the xArm using API calls 
"""

class Manipulator:

    def __init__(self):
        """Manipulator class initialization"""
        # initialize values
        self.pregrasp_offset = None
        self.ee_length_offset = None
        self.init_pose = None
        self.basket_pregrasp = None
        self.orientation = None
        self.ip = rospy.get_param('/xarm_robot_ip')
        arm_yaml = rospy.get_param('/arm_yaml')
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("peter")
        self.parseYaml(arm_yaml, package_path)

        # initialize xArm
        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.connect()
        self.arm.set_mode(0)
        self.arm.set_state(0)

    def parseYaml(self, yaml_file, package_path):
        """parse the yaml to get parameters"""
        # parse data
        data = dict()
        with open(os.path.join(package_path + yaml_file), "r") as file:
            data = yaml.safe_load(file)
        
        # save values
        self.pregrasp_offset = data["pregrasp_offset"]
        self.ee_length_offset = data["ee_offset"]
        self.init_pose = data["init_pose"]
        self.basket_pregrasp = data["basket_pregrasp"]
        self.orientation = data["orientation"]

    def moveToInit(self):
        """move to initial position"""
        print("Moving to initial pose")
        self.arm.set_position(*self.init_pose, wait=True, speed=20)

    def cartesianMoveX(self,dist): # todo: test out set_servo_cartesian
        """cartesian move along x"""
        dist = dist*1000  # convert m to mm
        current_pos = self.arm.get_position()[1]
        current_pos[0] += dist # add to x
        print("Executing cartesian move")
        self.arm.set_position(*current_pos, wait=True, speed=10)

    def cartesianMoveY(self,dist): # todo: test out set_servo_cartesian
        """cartesian move along y"""
        dist = dist*1000  # convert m to mm
        current_pos = self.arm.get_position()[1]
        current_pos[1] += dist # add to y
        print("Executing cartesian move")
        self.arm.set_position(*current_pos, wait=True, speed=10)

    def moveToPregrasp(self,x,y,z):
        """move to the poi pregrasp pose"""
        x -= self.pregrasp_offset
        x -= self.ee_length_offset
        # just using the orientation values from the init position
        self.arm.set_position(x * 1000 ,y * 1000 ,z * 1000 ,*self.orientation[3:], wait=True, speed=20)

    def moveToBasket(self):
        """move to basket pose for pepper drop off"""
        self.cartesianMoveX(-0.1) # move back 10 cm
        print("moving to basket pregrasp")
        self.arm.set_position(*self.basket_pregrasp, wait=True, speed=15) # move to basket pre-grasp
        self.cartesianMoveY(-0.15) # move forward to basket

    def moveToInitFromBasket(self):
        self.arm.load_trajectory('basket_to_init.traj')
        self.arm.playback_trajectory()

    def test(self):
        print("TESTING")
        # print(self.arm.get_position()[1])
        self.arm.set_position(*self.basket_pregrasp, wait=True, speed=15) # move to basket pre-grasp
        self.cartesianMoveY(-0.15) # move forward to basket
        print(self.arm.get_position()[1])
        rospy.sleep(20)
        # self.moveToInitFromBasket()
        # self.moveToInit()
        # self.moveToBasket()
        return
    
    def disconnect(self):
        """disconnect from xarm"""
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
            xarm.test()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
