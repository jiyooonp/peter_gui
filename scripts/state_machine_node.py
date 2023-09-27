#!/usr/bin/env python3

### ROS node for state machine
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

from xarm_msgs.srv import SetInt16, SetInt16Response

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
        # self.last_pressed = 0
        # self.button_counter = 0 

        # make a service server to be able to change the arm state
        self.xarm_mode_service = rospy.ServiceProxy("/xarm/set_mode", SetInt16)
        self.xarm_state_service = rospy.ServiceProxy("/xarm/set_state", SetInt16)


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
        # if self.last_pressed == self.joy_state: 
        #     self.button_counter += 1
        # else:
        #     self.button_counter = 0
        
        #if the button has been pressed for a while, change the state
        # if self.button_counter > 10:
            
        #  double square -> manual teleop
        # if (self.joy_state[6] or self.joy_state[2]) and self.state != 1:
        if self.joy_state[6] and self.state != 1:
            self.state = 1
            rospy.loginfo("Enter Manual Mode")

        # xbox button -> vs mode
        elif self.joy_state[8] and self.state != 2: 
            self.state = 2
            rospy.loginfo("Enter VS Mode")
            
        # Y -> enter idle mode
        elif self.joy_state[3] and self.state != 0:
            self.state = 0
            rospy.loginfo("Enter Idle Mode")                
        
    # def xarm_mode_service_call(self, mode):
        
    #     try:
            
    #         mode_response, state_response = None, None
    #         if mode == 0:
    #             mode_response = self.xarm_mode_service(0)
    #             state_response = self.xarm_state_service(4)
    #         else:
    #             mode_response = self.xarm_mode_service(0)
    #             state_response = self.xarm_state_service(0)
                
    #         if mode_response is not None and state_response.ret == mode_response.ret == 0:
    #             rospy.logwarn(f"\n{state_response.message} {mode_response.message}")  
    #             return True

    #         elif mode_response.ret != 0: 
    #             rospy.logwarn(f"Service mode call failed: {state_response.message}")               
    #             return False
            
    #         elif state_response.ret != 0:
    #             rospy.logwarn(f"Service state call failed: {state_response.message}")
    #             return False
            
    #     except rospy.ServiceException as e:
    #         rospy.logwarn("Service call failed: {}".format(e))
    #         return False

    def run(self):
        """Main loop"""

        while not rospy.is_shutdown():
            # publish state
            self.state_pub.publish(self.state)
            # sleep for 10 Hz
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        fsm = StateMachineNode()
        fsm.run()
    except rospy.ROSInterruptException:
        pass
