#!/usr/bin/python

import rospy
from rospy import Time
import numpy as np
from robot_module_msgs.srv import MoveVise
# Because of transformations
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import time
import argparse
import sys
import tf


class FixedTFBroadcaster:
    def __init__(self, cutter, frame, matrix):
        self.cutter = cutter
        self.frame = frame
        self.matrix = matrix

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    def tf_loop(self, cutter, frame, matrix):
        self.cutter = cutter
        self.frame = frame
        self.matrix = matrix

        for i in range(len(self.matrix)):
            self.matrix[i] = float(self.matrix[i])

        self.matrix[3:] = tf.transformations.quaternion_from_euler(self.matrix[3], self.matrix[4], self.matrix[5])
        

        while not rospy.is_shutdown():
            # Run this loop at about 100Hz
            rospy.sleep(0.01)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = str(self.frame)
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = str(self.cutter)
            t.transform.translation.x = self.matrix[0]
            t.transform.translation.y = self.matrix[1]
            t.transform.translation.z = self.matrix[2]

            t.transform.rotation.x = self.matrix[3]
            t.transform.rotation.y = self.matrix[4]
            t.transform.rotation.z = self.matrix[5]
            t.transform.rotation.w = self.matrix[6]

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

        
    def move_cutter(self, trigger):
        try:
            if trigger.trigger == True:
                self.matrix[2] = 0.517
            elif trigger.trigger == False:
                self.matrix[2] = 0.66
                
            return True, "Cutter: {} moved!".format(self.cutter)
        except:
            return False, "Wrong input!"


if __name__ == '__main__':
    rospy.init_node('move_cutter_blade', anonymous=True)    
    parser = argparse.ArgumentParser()
    parser.add_argument('--cutter', default=None, type=str)
    parser.add_argument('--frame', default=None, type=str)
    parser.add_argument('--matrix', nargs='+', default=[0,0,0,0,0,0])

    try:
        args, unknown = parser.parse_known_args(rospy.myargv()[:])
        rospy.loginfo(args)
        tfb = FixedTFBroadcaster("", "", None)
        rospy.loginfo("Move vise tool service has started...")
        # service move_cutter/cutter 
        service_path = "/move_cutter/" + str(args.cutter)
        service = rospy.Service(service_path, MoveVise, tfb.move_cutter)
        time.sleep(1)
        tfb.tf_loop(args.cutter, args.frame, args.matrix)
    except Exception as e:
        rospy.loginfo("Error: {}".format(e))

    rospy.spin()

