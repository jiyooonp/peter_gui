### ROS node for state machine
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

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

    def joystick_callback(self, data):
        """Callback for joystick message"""
        # update joy state
        self.joy_state = data
        # check if any of the joystick buttons are pressed
        # if A button is pressed go back to init position
        
        # TODO choose manual and teleop switch button is pressed, go to state 1
        # TODO if auto visual servo button is pressed, go to state 2

        #TODO - Alec: goal, switch states with an exotic button
        pass
        

    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            # publish state
            self.state_pub.publish(self.state)
            # sleep for 10 Hz
            rospy.sleep(0.1)
