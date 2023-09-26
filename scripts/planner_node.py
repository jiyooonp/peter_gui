#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from xarm_msgs.msg import RobotMsg


"""
Listen to state message. 
Depending on state, call the appropriate callback function and run the corresponding planner. 
"""

class PlannerNode:
    def __init__(self):

        # initialize values
        rospy.init_node('planner_node', anonymous=True)
        self.state = 0
        self.joy_state = Joy()
        self.fake_joy = Joy()
        self.visual_servoing_state = Twist()

        # initial fake joy values
        self.fake_joy.header.frame_id = "/dev/input/js0"
        self.fake_joy.header.stamp = rospy.Time.now()
        self.fake_joy.axes = [0 for _ in range(0,8)]
        self.fake_joy.buttons = [0 for _ in range(0,11)]

        self.joy_state.header.frame_id = "/dev/input/js0"
        self.joy_state.header.stamp = rospy.Time.now()
        self.joy_state.axes = [0 for _ in range(0,8)]
        self.joy_state.buttons = [0 for _ in range(0,11)]

        # subscribers
        self.state_sub = rospy.Subscriber('/state', Int16, self.state_callback, queue_size=1) # state message
        self.joystick_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1) # joystick message        
        self.visual_servoing_sub = rospy.Subscriber('/cmd_vel', Twist, self.visual_servo, queue_size=1) #visual servoing messages
        self.xarm_sub = rospy.Subscriber('/xarm/xarm_states', RobotMsg, self.robot_state_callback, queue_size=1) # joystick message
        
        # publishers
        self.joy_pub = rospy.Publisher('/joy_relay', Joy, queue_size=1) # joystick commands pub

    def send_to_xarm(self, val):
        """discretize arm teleop value"""
        if val > 0.1:
            return 1
        elif val < -0.1:
            return -1
        else:
            return 0

    def state_callback(self, data):
        """Callback for state message"""
        self.state = data.data
    
    def robot_state_callback(self, data):
        """Callback for robot state message"""
        self.ee_pose = data.pose
        # print(f"ee pose: {self.ee_pose}")

    def joystick_callback(self, data):
        """Callback for joystick message"""
        # update joy state
        self.joy_state = data


    def visual_servo(self, data):
        """visual servoing planner - convert twist to joy"""
        self.visual_servoing_state = data

        self.fake_joy.header.stamp = rospy.Time.now()
        self.fake_joy.axes = [0 for _ in range(0,8)]
        self.fake_joy.buttons = [0 for _ in range(0,11)]

        # cartesian move forward if X is pressed
        if (self.joy_state.buttons[2] == 1):
            self.fake_joy.axes[4] = 0.5 # forward is positive x
            return

        # get the visual servo values
        dy = self.visual_servoing_state.linear.x  # horizontal
        dz = self.visual_servoing_state.linear.y  # vertical
        dx = self.visual_servoing_state.linear.z  # depth
        # print(f"dx: {dx}, dy: {dy}, dz: {dz}")
   
        y =  dy * math.cos(math.radians(45)) + dz * math.cos(math.radians(45))
        z = -dy * math.sin(math.radians(45)) + dz * math.cos(math.radians(45))

        # update the fake joy message to publish
        self.fake_joy.axes[4] = self.send_to_xarm(dx)   # forward/back
        self.fake_joy.axes[6] = -self.send_to_xarm(y)   # left/right
        self.fake_joy.axes[7] = -self.send_to_xarm(z)   # up/down

        print(f"forward/back: {self.fake_joy.axes[4]}")
        print(f"left/right: {self.fake_joy.axes[6]}")
        print(f"up/down: {self.fake_joy.axes[7]}")

        return


    def run(self):
        """check what state the robot is in and run the corresponding planner"""
        # idle state
        print(self.state)
        if self.state == 0:
            # print("idle state")
            pass
        
        # manual teleop state (forward joystick messages to arm and EE)
        elif self.state == 1:
            # print("manual teleop state")
            self.joy_pub.publish(self.joy_state) # publish real joy arm
        
        # auto visual servo state
        elif self.state == 2:
            # self.fake_joy.header.stamp = rospy.Time.now()
            # self.fake_joy.axes = [0 for _ in range(0,8)]
            # self.fake_joy.buttons = [0 for _ in range(0,11)]
            # # cartesian move forward if X is pressed
            # if (self.joy_state.buttons[2] == 1):
            #     self.fake_joy.axes = [0 for _ in range(0,8)]
            #     self.fake_joy.axes[4] = 0.5 # forward is positive x
            self.joy_pub.publish(self.fake_joy) # publish fake joy to arm


if __name__ == '__main__':

    try:
        planner_node = PlannerNode()
        while not rospy.is_shutdown():
            planner_node.run()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
