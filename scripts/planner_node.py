#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from xarm_msgs.msg import RobotMsg
from manipulator import Manipulator
from ag_gripper_driver.srv import Pegasus, PegasusResponse


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
        self.planner_state_pub = rospy.Publisher('/planner_state', bool, queue_size=1) # planner state pub
        self.state_pub = rospy.Publisher('/state', Int16, queue_size=1) # state pub

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
    
    def send_to_ee(self, command):
        # send commands to open, harvest, or close
        # need to wait for confirmation this is done before returning

        rospy.wait_for_service('/gripper_service')
        if command == "open":
            try:
                Pegasus_action = rospy.ServiceProxy('/gripper_service',Pegasus)
                Pegasus_action(1)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        elif command == "harvest":
            try:
                Pegasus_action = rospy.ServiceProxy('/gripper_service',Pegasus)
                Pegasus_action(0)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        return
        

    def run(self):
        """check what state the robot is in and run the corresponding actions"""

        # idle
        if self.state == 0:
            pass

        # teleoperation - manual only
        elif self.state == 1:
            self.joy_pub.publish(self.joy_state) # publish real joy arm and ee

        # visual servoing - manual only
        elif self.state == 2:
            self.joy_pub.publish(self.fake_joy) 

        # return to init - manual only
        elif self.state == 3:
            self.manipulator.moveToInit()

        # ----- Initializing Autonomous Procedure -----
        # move to init position 
        elif self.state == 4:
            self.planner_state_pub(False)
            self.manipulator.moveToInit() 
            rospy.loginfo("Plan Execution: Initalization Complete")
            self.planner_state_pub(True)

        # visual servo to 10 cm in front of plant
        elif self.state == 5:
            self.planner_state_pub(False)
            self.joy_pub.publish(self.fake_joy) 
            rospy.loginfo("Plan Execution: Visual Servoing Complete")

            #TODO: NEED TO DETERMINE END CONDITION HERE
            self.planner_state_pub(True)

        # move to pregrasp position: open ee aplace ee at cut/grip position
        elif self.state == 6:
            self.planner_state_pub(False)
            self.send_to_ee("open")
            self.manipulator.cartesianMove(0.1)
            self.planner_state_pub(True)

        # harvest pepper
        elif self.state == 7:
            self.planner_state_pub(False)
            self.send_to_ee("harvest")
            self.planner_state_pub(True)

        # move to basket and drop
        elif self.state == 8:
            self.planner_state_pub(False)
            self.manipulator.moveToBasket(0.1)
            self.send_to_ee("open")
            self.manipulator.moveToInit()
            self.planner_state_pub(True)

        else:
            rospy.loginfo("ERROR: UNRECOGNIZED STATE IN PLANNER NODE")
            self.state_pub.publish(10)


if __name__ == '__main__':

    try:
        planner_node = PlannerNode()
        while not rospy.is_shutdown():
            planner_node.run()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
