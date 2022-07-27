#!/usr/bin/env python
PACKAGE = 'radiation_layer_isra'
import rospy
from sensor_msgs.msg import Image, CameraInfo, geometry_msgs, PointCloud2, PointCloud, PointField
from cv_bridge import CvBridge
import cv2
import tf
import image_geometry
import message_filters
import geometry_msgs.msg
import std_msgs.msg
import numpy as np
import math
import struct
import matplotlib.pyplot as plt
import time
from geometry_msgs.msg import Point32
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from kinect_opencv.msg import StampedArray
import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {lower_threshold}, {lower_threshold_scale}, {upper_threshold}, {upper_threshold_scale}".format(**config))


def GetNewRadThreshold(robot_rad):
    '''
        This function updates the threshold relating to radiation by scaling the threshold based on the robot's state. 
        Maximum radiation is 1000mGy and the measure is in mGy, so we need to scale the threshold as a percentage of 253 (which is maximum cost)
        using the robot's dose data.
    '''

    #this calculates the robot's state as a percentage of the maximum
    state_percentage = robot_rad/1000

    #this sets the threshold to be proportional to the robot's state
    new_threshold = 253 - 253*state_percentage

    if new_threshold < 1:
        new_threshold = 1

    return(new_threshold)


def process_data(state):
    try:
        robot_rad = state.array[0]

        updated_rad_threshold = GetNewRadThreshold(robot_rad)


        # updates the configuration of the server with new data 
        # currently this just sends all the updates to the radiation configuration file for testing, waiting on Andy for to instigate temperature.
        #TODO: Add extra configuration update for temperature, when that layer is working.
        clientLocal.update_configuration({"upper_threshold":updated_rad_threshold})
        clientGlobal.update_configuration({"upper_threshold":updated_rad_threshold})


    except Exception as err:
            print err

    



def start_node():
    '''
        This function is run when the program starts, after declaration of the publisher. 
        It initialises the node and subscribes to the relevant topics. It then spins until
        the node is terminated.
    '''

    #Initialise this node
    rospy.init_node("radiation_layer_dynamic_client")
    rospy.loginfo('radiation_layer_dynamic_client node started')
    
    #Subscribe to the camera, point cloud and camera info topics
    state_sub = rospy.Subscriber("/robot_state_publisher", StampedArray, process_data)


    rospy.spin()

if __name__ == "__main__":
    
    try:
        #client = dynamic_reconfigure.client.Client("Radiation_Layer_Server", timeout=30)
        clientLocal = dynamic_reconfigure.client.Client("/move_base/local_costmap/radiationLocal", timeout=30)
        clientGlobal = dynamic_reconfigure.client.Client("/move_base/global_costmap/radiationGlobal", timeout=30)
        start_node()
    except rospy.ROSInterruptException:
        pass

