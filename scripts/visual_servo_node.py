#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist


class VisualServoingNode:
    def __init__(self):

        rospy.init_node('visual_servoing_node', anonymous=True)
        self.rate = rospy.Rate(30)

        # RealSense image dimensions
        self.image_width = 640.0
        self.image_height = 480.0

        # Set target x and y pixels (RealSense coordinate system)
        self.target_x = self.image_width/2  # 425
        self.target_y = self.image_height/2 # 135

        # Set control gains
        self.k_x = 2
        self.k_y = 2
        self.k_z = 1

        # pepper center and peduncle center
        self.pepper_center = [self.target_x, self.target_y, 0]
        self.peduncle_center = [self.target_x, self.target_y, 0]

        # Subscribe to perception poi message
        self.peduncle_center_sub = rospy.Subscriber(
            '/peduncle_center', Point, self.peduncle_center_callback)

        self.pepper_center_sub = rospy.Subscriber(
            '/pepper_center', Point, self.pepper_center_callback)

        # Publish a command velocity
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


    def pepper_center_callback(self, data):
        self.pepper_center = [data.x, data.y, data.z]
        print(":pepper:", self.peduncle_center)
       

    def peduncle_center_callback(self, data):
        self.peduncle_center = [data.x, data.y, data.z]
        print(":peduncle:", self.peduncle_center)
        

    def publish_velocity(self):

        # Get actual location of poi
        # No detections of both pepper and peduncle so don't move
        if self.peduncle_center == [-1, -1, -1] and self.pepper_center == [-1, -1, -1]:
            actual_location = [self.target_x, self.target_y, 0]
        # Peduncle detected so move to peduncle
        elif self.peduncle_center != [-1, -1, -1]:
            actual_location = self.peduncle_center
        # Only pepper detected so move to pepper
        else:
            actual_location = self.pepper_center
        
        # Calculate error between poi and image center
        target_error_x = self.target_x - actual_location[0]
        target_error_y = self.target_y - actual_location[1]

        # Normalize errors
        target_error_x = target_error_x/self.image_width
        target_error_y = target_error_y/self.image_height

        # Visual servoing control law (velocities set according to RealSense frame)
        vel_cmd = Twist()
        vel_cmd.linear.x = max(min(self.k_x * target_error_x, 1), -1)
        vel_cmd.linear.y = max(min(self.k_y * target_error_y, 1), -1)
        vel_cmd.linear.z = max(min(self.k_z * actual_location[2], 1), -1)

        # Publish velocity
        print(f"vel_cmd: {vel_cmd}")
        self.cmd_vel_pub.publish(vel_cmd)


    def run(self):
        while not rospy.is_shutdown():
            self.publish_velocity()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        visual_servoing_node = VisualServoingNode()
        visual_servoing_node.run()
    except rospy.ROSInterruptException:
        pass
