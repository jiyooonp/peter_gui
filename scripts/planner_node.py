import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import math

"""
Listen to state message. 
Depending on state, call the appropriate callback function and run the corresponding planner. 
"""

class PlannerNode:
    def __init__(self):
        rospy.init_node('planner_node', anonymous=True)
        self.state = 0
        self.joy_state = Joy()
        self.fake_joy = Joy()
        self.visual_servoing_state = Twist()

        # subscribers
        self.state_sub = rospy.Subscriber('/state', Int16, self.state_callback, queue_size=1) # state message
        self.joystick_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1) # joystick message        
        self.visual_servoing_sub = rospy.Subscriber('/visual_servo', Twist, self.visual_servoing_callback, queue_size=1) #visual servoing messages

        # publishers
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=1) # joystick commands pub (which dom's teleop script is subscribed to)
        self.ee_joy_pub = rospy.Publisher('/ee_joy', Joy, queue_size=1)      # forwarding joystick commands to ee # todo: update topic name
        self.arm_pub = rospy.Publisher('/arm_control', Int16, queue_size=1)  # arm control # todo: when should this be used?
        self.ee_pub = rospy.Publisher('/ee_control', Int16, queue_size=1)    # publisher for EE control # todo: when should this be used?

    def state_callback(self, data):
        """Callback for state message"""
        self.state = data.data
    
    def joystick_callback(self, data):
        """Callback for joystick message"""
        # update joy state
        self.joy_state = data

    def visual_servoing_callback(self, data):
        """Callback for visual servoing message"""
        self.visual_servoing_state = data

    # TODO: implement the following function -- Sri
    def visual_servo(self):
        """visual servoing planner"""
        # convert twist message to a joystick command and then publish the joystick command to the arm and EE

        # get the visual servo values
        dy = self.visual_servoing_state.linear.x  # horizontal
        dz = self.visual_servoing_state.linear.y  # vertical
        dx = self.visual_servoing_state.linear.z  # depth

        # set scale factors
        x_scale = 1
        y_scale = 1
        z_scale = 1
        
        # update the fake joy message to publish

        self.fake_joy.axes[4] = x_scale * dx  # forward/back button on joystick

        # move left/right
        self.fake_joy.axes[6] = y_scale * dy * math.cos(math.pi/4)    # left/right
        self.fake_joy.axes[7] = y_scale * dy * math.cos(math.pi/4)   # up/down

        # move up and down
        self.fake_joy.axes[6] += z_scale * dz * math.cos(math.pi/4)    # left/right
        self.fake_joy.axes[7] += z_scale * dz * math.cos(math.pi/4)   # up/down

        return
    

    def run(self):
        """check what state the robot is in and run the corresponding planner"""
        # idle state
        if self.state == 0:
            print("idle state")
            return
        
        # manual teleop state
        elif self.state == 1:
            # simply forward joystick messages to arm and EE
            print("manual teleop state")
            self.ee_joy_pub.publish(self.joy_state) # publish to ee
            self.joy_pub.publish(self.joy_state) # publish to arm
            return
        
        # auto visual servo state
        elif self.state == 2:
            print("visual servo state")
            self.joy_pub.publish(self.fake_joy)
            return
