#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

def publish_numbers():
    rospy.init_node('number_publisher', anonymous=True)
    pub = rospy.Publisher('/state', Int16, queue_size=10)
    rate = rospy.Rate(0.2)  # 0.2 Hz, equivalent to a 5-second delay

    number = 2
    while not rospy.is_shutdown():
        msg = Int16()
        msg.data = number
        pub.publish(msg)
        rospy.loginfo("Publishing number: %d", number)
        
        number += 1
        if number > 10:
            number = 1

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_numbers()
    except rospy.ROSInterruptException:
        pass
