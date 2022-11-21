#!/usr/bin/python

import rospy
from rospy import Time
import numpy as np

# Because of transformations
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

import time
import argparse
import sys
import tf


class RosNode:
    def __init__(self, joint_name, subscriber_topic, publisher_topic, min_value, max_value, move_time):

        self.refresh_rate = 0.1
        self.joint_name = joint_name
        self.pulz_end_N = int(move_time/self.refresh_rate)
        self.value_step = (max_value - min_value)/self.pulz_end_N
        self.direction_of_move = -1
        
        self.current_value = 0
        self.min_value = min_value
        self.max_value = max_value


        self.pub_joint = rospy.Publisher(publisher_topic, JointState, queue_size=1)
        self.sub_bool = rospy.Subscriber(subscriber_topic, Bool, self.subscriber_loop)
        self.timer = rospy.Timer(rospy.Duration(self.refresh_rate), self.timer_loop)
        #self.timer.shutdown()

    def subscriber_loop(self, msg):

        if msg.data:
            self.direction_of_move = 1
        else:
            self.direction_of_move = -1


        #self.timer.start()
   
        
    def timer_loop(self,event):

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [self.joint_name]

        self.current_value = self.current_value + self.direction_of_move*self.value_step

        if self.current_value<=self.min_value:
            self.current_value = self.min_value
            #self.timer.shutdown()


        elif self.current_value>=self.max_value:
            self.current_value = self.max_value
            #self.timer.shutdown()
        rospy.loginfo(self.current_value)

        joint_state.position = [self.current_value]
        
        self.pub_joint.publish(joint_state)






if __name__ == '__main__':
    rospy.init_node('vis_joint_publisher', anonymous=True)    
    parser = argparse.ArgumentParser()
    parser.add_argument('--joint_name', default=None, type=str)
    parser.add_argument('--subscribe_topic', default=None, type=str)
    parser.add_argument('--publish_topic', default=None, type=str)
    parser.add_argument('--min_value', type=float, default=0.0)
    parser.add_argument('--max_value', type=float, default=1.0)
    parser.add_argument('--move_time', type=float, default=10.0)


    try:
        args, unknown = parser.parse_known_args(rospy.myargv()[:])
        rospy.loginfo(args)
        node = RosNode(args.joint_name, args.subscribe_topic, args.publish_topic, args.min_value, args.max_value, args.move_time)
        rospy.loginfo("Vis publisher started: "+args.joint_name)

    except Exception as e:
        rospy.loginfo("Error: {}".format(e))

    rospy.spin()

