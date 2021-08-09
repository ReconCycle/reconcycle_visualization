#!/usr/bin/python

import rospy
from rospy import Time
import numpy as np
from robot_module_msgs.srv import rviz_tools, SetList
# Because of transformations
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import time
import argparse
import sys
import tf


class FixedTFBroadcaster:
    def __init__(self, tool, frame, Tmatrix):
        self.tool = tool
        self.frame = frame
        self.matrix = Tmatrix

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    def tf_loop(self, tool, frame, Tmatrix):
        self.tool = tool
        self.frame = frame
        self.matrix = Tmatrix

        for i in range(len(self.matrix)):
            self.matrix[i] = float(self.matrix[i])

        self.matrix[3:] = tf.transformations.quaternion_from_euler(self.matrix[3], self.matrix[4], self.matrix[5])
        

        while not rospy.is_shutdown():
            # Run this loop at about 100Hz
            rospy.sleep(0.01)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = str(self.frame)
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = str(self.tool)
            t.transform.translation.x = self.matrix[0]
            t.transform.translation.y = self.matrix[1]
            t.transform.translation.z = self.matrix[2]

            t.transform.rotation.x = self.matrix[3]
            t.transform.rotation.y = self.matrix[4]
            t.transform.rotation.z = self.matrix[5]
            t.transform.rotation.w = self.matrix[6]

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

    def service_call(self, tool_data):
        if self.frame == tool_data.frame:
            rospy.loginfo("Tool is already on this position!")
            return False, "Tool is already on this position!"
        else:
            self.frame = tool_data.frame
            return True, "Tool moved"
        
    def change_T_matrix(self, T_data):
        try:
            self.matrix = list(T_data.matrix)
            for i in range(len(self.matrix)):
                self.matrix[i] = float(self.matrix[i])
                
            self.matrix[3:] = tf.transformations.quaternion_from_euler(self.matrix[3], self.matrix[4], self.matrix[5])
            return True, "Transform changed to {} !".format(self.matrix)
        except:
            self.matrix = [0, 0, 0, 0, 0, 0, 1]
            return False, "Wrong input, matrix set to default value: {}!".format(self.matrix)


if __name__ == '__main__':
    rospy.init_node('tool_viz_node', anonymous=True)    
    parser = argparse.ArgumentParser()
    parser.add_argument('--tool', default=None, type=str)
    parser.add_argument('--frame', default=None, type=str)
    parser.add_argument('--Tmatrix', nargs='+', default=[0,0,0,0,0,0])

    try:
        args, unknown = parser.parse_known_args(rospy.myargv()[:])
        rospy.loginfo(args)
        tfb = FixedTFBroadcaster("", "", None)
        rospy.loginfo("Tool change service has started...")
        service_path = "/change_tool_frame/" + str(args.tool)

        service = rospy.Service(service_path, rviz_tools, tfb.service_call)

        T = "/change_transform_matrix/" + str(args.tool)
        T_service = rospy.Service(T, SetList, tfb.change_T_matrix)
        time.sleep(1)
        tfb.tf_loop(args.tool, args.frame, args.Tmatrix)
    except:
        rospy.loginfo("Error")

    rospy.spin()

