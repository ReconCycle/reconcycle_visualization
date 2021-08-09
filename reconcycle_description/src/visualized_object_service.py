#!/usr/bin/env python

import rospy
import tf
from tf import TransformBroadcaster
import numpy as np
from rospy import Time
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from robot_module_msgs.srv import ToolChanger
import time
import collections
import os
import subprocess
import signal

# tf broadcaster
tfsender = TransformBroadcaster()
# global variables
import_object_name = ''
import_object_ns = ''
attach_to_panda = 'panda_1/panda_link8'
objectScaleZ = 0.03
camera_1_markers = []
executer = False

# variables
tf_rotation = [0, 0, 0]
Coords = [0, 0, 0]
ZOffset = 0
scale = 0.01

# Subscriber
subs = None
# Publisher
markerPub = None


# Subscriber @object_to_hand_service_call calling call_marker funcion
def call_marker(marker_array_data):
    global camera_1_markers
    wait_for_camera_1 = True

    object_marker = marker_array_data.markers
    
    camera_1_markers = []
    
    while wait_for_camera_1:
        for i in range(len(object_marker)):
            if object_marker[i].ns == "/camera_1/object":
                camera_1_markers.append(object_marker[i])
            if len(camera_1_markers) == len(object_marker):
                wait_for_camera_1 = False
                rospy.loginfo("{}".format(camera_1_markers))
            
    
    subs.unregister()   
    return True

# service call function
def object_to_hand_service_call(tools_data):
    global import_object_name
    global import_object_ns
    global attach_to_panda
    global executer
    global subs

    # no changes if inputs are empty
    if tools_data.toolname == '':
        pass
    else:
        tool_name = tools_data.toolname

    if tools_data.groupname == '':
        pass
    else:
        group_name = tools_data.groupname

    if tools_data.transform_to == '':
        pass
    else:
        attach_to_panda = tools_data.transform_to
    
    if tools_data.hold == '':
        pass
    else:
        executer = tools_data.hold

    # msg_temp wait_for_message is used in case no object is present and service is called.
    # Service waits until object data is available
    rospy.loginfo("Received service call")   
    msg_temp = rospy.wait_for_message("/visualization_marker_array", MarkerArray, timeout=5)
    time.sleep(0.5)
    # When topic data is available then subscribing starts
    subs = rospy.Subscriber("/visualization_marker_array", MarkerArray, call_marker)
    rospy.loginfo("Data on /visualisation_marker_array are available on topic! Executing function")   
    

    return [True,"Success"]

def active_data(marker_data):
    marker = Marker()
    marker_length = len(marker_data)
    marker = marker_data
    global executer

    try:
        if executer == True:
            if marker_length > 0:
                for i in range(marker_length):
                    translation = (Coords[0], Coords[1], Coords[2] + ZOffset * scale)       
                    rotation = tf_rotation
                    rotation = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
                    tfsender.sendTransform(translation, rotation, Time.now(), marker[i].header.frame_id, str(attach_to_panda))
                
                    # Prepare Marker message
                    marker[i].header.stamp = Time.now()
                    marker[i].text = marker[i].header.frame_id
                    marker[i].pose.position.z = 0.14
                    marker[i].pose.position.y = 0.06
                    marker[i].pose.position.x = -0.02
                    marker[i].pose.orientation.z = 0
                    marker[i].pose.orientation.x = 1.05
                    markerPub.publish(marker[i])
            else:
                pass
    except ROSException:
        rospy.loginfo("No data to perform execution!")


if __name__ == '__main__':
    rospy.init_node("move_detected_objects")
    r = rospy.Rate(100)
    rospy.loginfo("Created /move_object_with_panda service node!")
    # ROS service server with ToolChanger data, calling object_to_hand function
    service_node = rospy.Service("/move_object_with_panda", ToolChanger, object_to_hand_service_call)
    rospy.loginfo("Service is running!")
    # marker publisher
    markerPub = rospy.Publisher('/visualization_marker', Marker, queue_size = 10)
    while not rospy.is_shutdown():
        active_data(camera_1_markers)
        r.sleep()