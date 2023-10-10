#!/usr/bin/env python3

import rospy
import numpy as np
import yaml
import tf2_ros
import geometry_msgs.msg
import tf_conversions

"""
broadcase yaml
"""

class TfBroadcaster:

    def __init__(self):
        # initialize values
        rospy.init_node("tf_handler_node")
        tf_yaml = rospy.get_param('/tf_yaml')
        
        self.parseYaml(tf_yaml)

        
    def broadcast(self):

        rot = self.H[0:3, 0:3]
        quaternion = tf_conversions.transformations.quaternion_from_matrix(rot)
        trans = self.H[0:3, 3]
        
        br = tf2_ros.StaticTransformBroadcaster()
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "link_eef"
        transform.child_frame_id = "realsense_frame"
        transform.translation = trans
        transform.rotation = quaternion
        
        br.sendTransform(transform)
        

    def parseYaml(self, yaml_file):
        """parse the yaml to reveal all the relevant transforms"""
    
        # parse data 
        data = dict()
        with open(yaml_file, "r") as file:
            data = yaml.safe_load(file)
        
        # convert the 2d matrices to a list of transforms
        transforms = list()
        for tf_mat in data.values():
            transforms.append(np.array(tf_mat))
          
        # apply transforms on to the right of each other  
        HTM = np.diag(4)
        for tf in transforms:
            HTM = HTM @ tf   
            
        self.H = HTM      


if __name__ == '__main__':

    try:
        tfs = TfBroadcaster()
        while not rospy.is_shutdown():
            tfs.broadcast()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass