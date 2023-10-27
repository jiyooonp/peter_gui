#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import geometry_msgs.msg
import tf_conversions


"""
broadcast fake tf for a trip
"""

class TfBroadcaster:

    def __init__(self):
        # initialize values
        rospy.init_node("fake_tf")
        
        self.H = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]
        )
        
    def broadcast(self):
        # rot = self.H[:3, :3]
        quaternion = tf_conversions.transformations.quaternion_from_matrix(self.H)
        trans = self.H[:, -1]
        
        br = tf2_ros.StaticTransformBroadcaster()
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "link_base"
        transform.child_frame_id = "camera_color_optical_frame"
        
        transform.transform.translation.x = trans[0]
        transform.transform.translation.y = trans[1]
        transform.transform.translation.z = trans[2]

        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        br.sendTransform(transform)  


if __name__ == '__main__':

    try:
        tfs = TfBroadcaster()
        while not rospy.is_shutdown():
            tfs.broadcast()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass