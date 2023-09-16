### ROS node for state machine
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

from xarm_msgs.srv import SentInt16, SentInt16Reponse 
"""
Possible states:
idle: 0
manual teleop: 1
auto visual servo: 2
return to init: 3
error: 4

"""
# TODO Alex & Solomon: finish implementing state machine node
class StateMachineNode:
    """ROS node for state machine"""
    def __init__(self):
        rospy.init_node('state_machine_node', anonymous=True)
        self.joy_state = Joy()
        # subscribe to joystick message
        self.joystick_callback = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1)
        # create publisher that contiously publishes state at specified rate
        self.state_pub = rospy.Publisher('/state', Int16, queue_size=1)
        self.state = 0 # initial state is 0 (idle)
        
        # how many times you receive the same button press
        self.last_pressed = 0
        self.button_counter = 0 
        
        # make a service server to be able to change the arm state
        self.xarm_service = rospy.ServiceProxy("/xarm/set_mode", SentInt16)

    def joystick_callback(self, data):
        """Callback for joystick message"""
        # update joy state
        self.joy_state = data.buttons
        # check if any of the joystick buttons are pressed
        # if A button is pressed go back to init position
        
        # TODO choose manual and teleop switch button is pressed, go to state 1
        # TODO if auto visual servo button is pressed, go to state 2
        
        '''
        Assumptions:
        - Only one button is pressed at a time
        '''
        
        # track if the button is being pressed for a while
        if self.last_pressed == self.joy_state: 
            self.button_counter += 1
        else:
            self.button_counter = 0
        
        #if the button has been pressed for a while, change the state
        if self.button_counter > 10:
            
            #  double square -> manual teleop
            if self.joy_state[6]:
                
                self.xarm_mode_service_call(1)
                
                self.state_pub(1)
                
            # xbox button -> vs mode
            elif self.joy_state[6]:
                
                self.xarm_mode_service_call(1)
                
                self.state_pub(2)
                
            # ~~~ -> enter idle mode
            if self.joy_state[6]:
                self.xarm_mode_service_call(0)
                
                self.state_pub(0)
                
        # track the most recent press
        self.last_pressed = self.joy_state
        
    def xarm_mode_service_call(self, mode):
        
        request = SentInt16()  
        request.data = mode # mode 0 does not accept move responses
        
        try:
            response = self.xarm_service(request)
            
            if response.ret == 0:
                rospy.loginfo(f"Successful Idle Entrance: {response.response}")
                
                self.state_pub.publish(0)
                
            else:
                rospy.logwarn(f"Failed to Enter Idle Mode: {response.response}")
            
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))

    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            # publish state
            self.state_pub.publish(self.state)
            # sleep for 10 Hz
            rospy.sleep(0.1)
