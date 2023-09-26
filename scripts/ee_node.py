#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from ag_gripper_driver.srv import Pegasus, PegasusResponse

class EENode:
    def __init__(self) -> None:
        rospy.init_node('ee_node', anonymous=True)
        self.joy_state = Joy()
        self.joystick_callback = rospy.Subscriber('/joy_relay', Joy, self.joystick_callback, queue_size=1)


 # TODO move to EE node
    def joystick_callback(self, data):
        self.joy_state = data
        # check if any of the joystick buttons are pressed
        # if A button is pressed go back to init position
        if self.joy_state.buttons[0] == 1:
            rospy.wait_for_service('/gripper_service')
            try:
                 cutter = rospy.ServiceProxy('/gripper_service',Pegasus)
                 # pass args to service
                 cutter(0)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            return


        # if LB button is pressed open gripper and cutter service
        if self.joy_state.buttons[4] == 1:
            rospy.wait_for_service('/gripper_service')
            try:
                 cutter = rospy.ServiceProxy('/gripper_service',Pegasus)
                 # pass args to service
                 cutter(1)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            return

        # if RB button is pressed call gripper service
        if self.joy_state.buttons[5] == 1:
            rospy.wait_for_service('/gripper_service')
            try:
                 cutter = rospy.ServiceProxy('/gripper_service',Pegasus)
                 # pass args to service
                 cutter(2)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            return
        

# Listen to planner
    # service calls to end effector driver

# Listen to state machine
    # act appropiately based on if errror state or mid - visual servoing

if __name__ == '__main__':

    try:
        EE_node = EENode()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
       