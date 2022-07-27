#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo, geometry_msgs, PointCloud2, PointCloud
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
from rospy_tutorials.msg import Floats
from kinect_opencv.msg import StampedArray

        
def process_pointClouds(front_sub, back_sub):#, left_sub, right_sub):
    ''' 
        This function is the callback function that is run when data is received on the 
        point cloud topics. It takes point clouds as input. It then processes
        this information and publishes a unified point cloud.
    '''

    
    try:      
        print("running")
        #Declare point cloud
        # pointCloudOut = PointCloud()   

        # #filling pointcloud header
        # header = std_msgs.msg.Header()
        # header.stamp = rospy.Time.now()
        # header.frame_id = '/odom' #explains what frame the point cloud data is in
        # pointCloudOut.header = header 
        # #i = 0  

        # #filling the points with the world frame position
        # # for j in range(len(front_sub.array)/3):
        # #     pointCloudOut.points.append(Point32(front_sub.array[i], front_sub.array[i+1], front_sub.array[i+2]))
        # #     i=i+3
        
        # # i=0
        # # for j in range(len(back_sub.array)/3):
        # #     pointCloudOut.points.append(Point32(back_sub.array[i], back_sub.array[i+1], back_sub.array[i+2]))
        # #     i=i+3

        # # i=0
        # # for j in range(len(left_sub.array)/3):
        # #     pointCloudOut.points.append(Point32(left_sub.array[i], left_sub.array[i+1], left_sub.array[i+2]))
        # #     i=i+3

        # # i=0
        # # for j in range(len(right_sub.array)/3):
        # #     pointCloudOut.points.append(Point32(right_sub.array[i], right_sub.array[i+1], right_sub.array[i+2]))
        # #     i=i+3



        # pointcloud_publisher.publish(pointCloudOut)

    except Exception as err:
            print err

def start_node():
    '''
        This function is run when the program starts, after declaration of the publisher. 
        It initialises the node and subscribes to the relevant topics. It then spins until
        the node is terminated.
    '''

    #Initialise this node
    rospy.init_node('radiation_scan_unifier')
    rospy.loginfo('radiation_scan_unifier node started')
    
    #Subscribe to the camera, point cloud and camera info topics
    front_sub = message_filters.Subscriber("/temperature_front_pointcloud_topic", StampedArray)

    back_sub = message_filters.Subscriber("/temperature_back_pointcloud_topic", StampedArray) ## TODO: CHANGE THIS TO BE RADIATION AGAIN

    #left_sub = message_filters.Subscriber("/temperature_left_pointcloud_topic", StampedArray)

    #right_sub = message_filters.Subscriber("/temperature_right_pointcloud_topic", StampedArray)
    
    #Ensure data is time synchronised and send to the call back function
    ts = message_filters.TimeSynchronizer([front_sub, back_sub],10)#, left_sub, right_sub], 100)
    ts.registerCallback(process_pointClouds)
    rospy.spin()

if __name__ == '__main__':
    try:
        pointcloud_publisher = rospy.Publisher("/unified_pointcloud_topic", PointCloud, queue_size = 10)
        start_node()
    except rospy.ROSInterruptException:
        pass