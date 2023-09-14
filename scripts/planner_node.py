import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

"""
Listen to state message. 
Depending on state, call the appropriate callback function and run the corresponding planner. 
"""

class PlannerNode:
    def __init__(self):
        rospy.init_node('planner_node', anonymous=True)        
        # subscribe to state message
        self.state = 0
        self.state_sub = rospy.Subscriber('/state', Int16, self.state_callback, queue_size=1)
        # subscribe to joystick message
        self.joy_state = Joy()
        self.joystick_callback = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1)
        # subscribe to visual servoing message
        self.visual_servoing_state = Twist()
        self.visual_servoing_sub = rospy.Subscriber('/visual_servo', Twist, self.visual_servoing_callback, queue_size=1)
        
        # publisher for arm control
        self.arm_pub = rospy.Publisher('/arm_control', Int16, queue_size=1)
        # publisher for EE control
        self.ee_pub = rospy.Publisher('/ee_control', Int16, queue_size=1) 

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

    # TODO implement the following function -- Sri 
    def visual_servo(self):
        """visual servoing planner"""
        # TODO implement visual servoing planner
        # convert twist message to a joystick command and then publish the joystick command to the arm and EE
        # TODO  
        return
    

    def run(self):
        """check what state the robot is in and run the corresponding planner"""
        if self.state == 0:
            # idle state
            print("idle state")
            return
        elif self.state == 1:
            # manual teleop state
            # simply forward joystick messages to arm and EE
            print("manual teleop state")
            return
        elif self.state == 2:
            # auto visual servo state
            
            return
