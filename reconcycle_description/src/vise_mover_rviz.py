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
    def __init__(self, vise, frame, matrix):
        self.vise = vise
        self.frame = frame
        self.matrix = matrix

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    def tf_loop(self, vise, frame, matrix):
        self.vise = vise
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
            t.child_frame_id = str(self.vise)
            t.transform.translation.x = self.matrix[0]
            t.transform.translation.y = self.matrix[1]
            t.transform.translation.z = self.matrix[2]

            t.transform.rotation.x = self.matrix[3]
            t.transform.rotation.y = self.matrix[4]
            t.transform.rotation.z = self.matrix[5]
            t.transform.rotation.w = self.matrix[6]

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

        
    def move_vise(self, trigger):
        try:
            if trigger.trigger == True:
                if self.vise == "vise_plate":
                    self.matrix[1] = -0.03                
                elif self.vise == "vise_slider":
                    self.matrix[0] = -0.215
                elif self.vise == "vise_rotation":
                    self.matrix[3] = 180
            elif trigger.trigger == False:
                if self.vise == "vise_plate":
                    self.matrix[1] = 0.14
                elif self.vise == "vise_slider":
                    self.matrix[0] = -0.26
                elif self.vise == "vise_rotation":
                    self.matrix[3] = 0
            
            self.matrix[3:] = np.deg2rad(self.matrix[3:])        
                
            self.matrix[3:] = tf.transformations.quaternion_from_euler(self.matrix[3], self.matrix[4], self.matrix[5])
            return True, "Vise {} moved! matrix: {}".format(self.vise, self.matrix)
        except:
            return False, "Wrong input!"


if __name__ == '__main__':
    rospy.init_node('move_vise_tool', anonymous=True)    
    parser = argparse.ArgumentParser()
    parser.add_argument('--vise', default=None, type=str)
    parser.add_argument('--frame', default=None, type=str)
    parser.add_argument('--matrix', nargs='+', default=[0,0,0,0,0,0])

    try:
        args, unknown = parser.parse_known_args(rospy.myargv()[:])
        rospy.loginfo(args)
        tfb = FixedTFBroadcaster("", "", None)
        rospy.loginfo("Move vise tool service has started...")
        # service move_vise_tool/$vise_tool_name -> exsisting tool names "vise_plate" "vise_slider" "vise_rotation"
        service_path = "/move_vise_tool/" + str(args.vise)
        service = rospy.Service(service_path, MoveVise, tfb.move_vise)
        time.sleep(1)
        tfb.tf_loop(args.vise, args.frame, args.matrix)
    except Exception as e:
        rospy.loginfo("Error: {}".format(e))

    rospy.spin()

