#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from xarm_msgs.msg import RobotMsg

import math

"""
Listen to state message. 
Depending on state, call the appropriate callback function and run the corresponding planner. 
"""

def send_to_xarm(val):
    if val > 0.1:
        return 1
    elif val < -0.1:
        return -1
    else:
        return 0
    
class PlannerNode:
    def __init__(self):
        rospy.init_node('planner_node', anonymous=True)
        self.state = 0
        self.joy_state = Joy()
        self.fake_joy = Joy()
        self.visual_servoing_state = Twist()

        self.fake_joy.header.frame_id = "/dev/input/js0"
        self.fake_joy.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        self.fake_joy.axes = [0 for _ in range(0,8)]
        self.fake_joy.buttons = [0 for _ in range(0,11)]

        # subscribers
        self.state_sub = rospy.Subscriber('/state', Int16, self.state_callback, queue_size=1) # state message
        self.joystick_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1) # joystick message        
        self.visual_servoing_sub = rospy.Subscriber('/cmd_vel', Twist, self.visual_servo, queue_size=1) #visual servoing messages
        # self.empty_message = rospy.Subscriber('/cmd_vel', Twist, self.visual_servo, queue_size=1) #visual servoing messages   
        self.xarm_sub = rospy.Subscriber('/xarm/xarm_states', RobotMsg, self.robot_state_callback, queue_size=1) # joystick message
        
        # publishers
        self.joy_pub = rospy.Publisher('/joy_relay', Joy, queue_size=1) # joystick commands pub
        self.ee_joy_pub = rospy.Publisher('/ee_joy', Joy, queue_size=1) # forwarding joystick commands to ee 
        # todo: update ee topic name

        # timer
        # timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

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
    
    # def timer_callback(self):
    #     self.joy_pub.publish(self.fake_joy)

    def visual_servo(self, data):
        """visual servoing planner"""
        # convert twist message to a joystick command and then publish the joystick command to the arm and EE
        self.visual_servoing_state = data

        self.fake_joy.header.stamp = rospy.Time.now()
        self.fake_joy.axes = [0 for _ in range(0,8)]
        self.fake_joy.buttons = [0 for _ in range(0,11)]

        # get the visual servo values
        dy = self.visual_servoing_state.linear.x  # horizontal
        dz = self.visual_servoing_state.linear.y  # vertical
        dx = self.visual_servoing_state.linear.z  # depth
        # print(f"dx: {dx}, dy: {dy}, dz: {dz}")
   
        y =  dy * math.cos(math.radians(45)) + dz * math.cos(math.radians(45))
        z = -dy * math.sin(math.radians(45)) + dz * math.cos(math.radians(45))

       # update the fake joy message to publish
        self.fake_joy.axes[4] = send_to_xarm(dx)  # forward/back button on joystick
        print(f"forward/back: {self.fake_joy.axes[4]}")

        # move left/right
        self.fake_joy.axes[6] = -send_to_xarm(y)    # left/right
        print(f"left/right: {self.fake_joy.axes[6]}")

        # move up/down
        self.fake_joy.axes[7] = -send_to_xarm(z)   # up/down
        print(f"up/down: {self.fake_joy.axes[7]}")

        return
    
    # def check_safety_limits(self):
    #     """check if the robot is within safety limits"""
    #     self.ee_pose

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
            self.ee_joy_pub.publish(self.joy_state) # publish to ee
        
        # auto visual servo state
        elif self.state == 2:
            # print("visual servo state")
            self.joy_pub.publish(self.fake_joy) # publish fake joy to arm
            self.ee_joy_pub.publish(self.joy_state) # publish to ee


if __name__ == '__main__':

    try:
        planner_node = PlannerNode()
        while not rospy.is_shutdown():
            planner_node.run()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
