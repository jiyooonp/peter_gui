#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from xarm_msgs.msg import RobotMsg
from manipulator import Manipulator


"""
Listen to state message. 
Depending on state, call the appropriate callback function and run the corresponding planner. 
"""

class PlannerNode:
    def __init__(self):

        # initialize values
        rospy.init_node('planner_node', anonymous=True)
        self.state = 0
        self.autonomous_phase = "init"
        self.joy_state = Joy()
        self.fake_joy = Joy()
        self.visual_servoing_state = Twist()
        self.manipulator = Manipulator()

        # initial fake joy values
        self.fake_joy.header.frame_id = "/dev/input/js0"
        self.fake_joy.header.stamp = rospy.Time.now()
        self.fake_joy.axes = [0 for _ in range(0,8)]
        self.fake_joy.buttons = [0 for _ in range(0,11)]

        # initial joy state values -> this is needed so we don't get an error when indexing joy state if it's empty
        self.joy_state.header.frame_id = "/dev/input/js0"
        self.joy_state.header.stamp = rospy.Time.now()
        self.joy_state.axes = [0 for _ in range(0,8)]
        self.joy_state.buttons = [0 for _ in range(0,11)]

        # subscribers
        self.state_sub = rospy.Subscriber('/state', Int16, self.state_callback, queue_size=1) # state message
        self.joystick_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1) # joystick message        
        self.visual_servoing_sub = rospy.Subscriber('/cmd_vel', Twist, self.visual_servo, queue_size=1) # visual servoing messages
        
        # publishers
        self.joy_pub = rospy.Publisher('/joy_relay', Joy, queue_size=1) # joystick commands pub

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

    def discretize(self, val):
        """discretize arm teleop value"""
        if val > 0.1:
            return 1
        elif val < -0.1:
            return -1
        else:
            return 0
        
    def visual_servo(self, data):
        """visual servoing planner - convert twist to joy"""
        self.visual_servoing_state = data

        self.fake_joy.header.stamp = rospy.Time.now()
        self.fake_joy.axes = [0 for _ in range(0,8)]
        self.fake_joy.buttons = [0 for _ in range(0,11)]

        # cartesian move forward if X is pressed
        if (self.joy_state.buttons[2] == 1):
            self.fake_joy.axes[4] = 1 # forward is positive x
            return
        
        # cartesian move to the -Y if B is pressed
        if (self.joy_state.buttons[1] == 1):
            self.fake_joy.axes[6] = 1/math.sqrt(2) # forward
            self.fake_joy.axes[7] = -1/math.sqrt(2) # up
            return
        
        # get the visual servo values
        dy = self.visual_servoing_state.linear.x  # horizontal
        dz = self.visual_servoing_state.linear.y  # vertical
        dx = self.visual_servoing_state.linear.z  # depth
        # print(f"dx: {dx}, dy: {dy}, dz: {dz}")
   
        y =  dy * math.cos(math.radians(45)) + dz * math.sin(math.radians(45))
        z =  -dy * math.cos(math.radians(45)) + dz * math.sin(math.radians(45))

        # update the fake joy message to publish
        self.fake_joy.axes[4] = self.discretize(dx)   # forward/back
        self.fake_joy.axes[6] = self.discretize(y)   # left/right
        self.fake_joy.axes[7] = self.discretize(z)   # up/down

        print(f"forward/back: {self.fake_joy.axes[4]}")
        print(f"left/right: {self.fake_joy.axes[6]}")
        print(f"up/down: {self.fake_joy.axes[7]}")

        return
    
    def send_to_ee(self):
        # send commands to open, harvest, or close
        # need to wait for confirmation this is done before returning
        return

    def run(self):
        """check what state the robot is in and run the corresponding actions"""

        # idle
        if self.state == 0:
            pass

        # manual teleop
        elif self.state == 1:
            self.joy_pub.publish(self.joy_state) # publish real joy arm and ee

        # autonomous mode
        elif self.state == 2:

            print(self.autonomous_phase)

            # move to init pose
            if self.autonomous_phase == "init":
                self.manipulator.moveToInit()
                self.autonomous_phase = "vs"
        
            # auto visual servo
            elif self.autonomous_phase == "vs":
                self.joy_pub.publish(self.fake_joy) # publish fake joy to arm
                # todo: need to determine end condition here
                self.autonomous_phase = "pregrasp"

            # pregrasp: open ee and cartesian move
            elif self.autonomous_phase == "pregrasp":
                # todo: open gripper/cutter and confirm it's done
                self.manipulator.cartesianMove(0.1)
                self.autonomous_phase = "harvest"

            # harvest pepper
            elif self.autonomous_phase == "harvest":
                # todo: ee harvest
                self.autonomous_phase = "basket"

            # basket drop
            elif self.autonomous_phase == "basket":
                self.manipulator.moveToBasket(0.1)
                # todo: open gripper
                self.manipulator.moveToInit()
                self.autonomous_phase = "done"

            else:
                print("done with autonomous harvesting sequence")



if __name__ == '__main__':

    try:
        planner_node = PlannerNode()
        while not rospy.is_shutdown():
            planner_node.run()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
