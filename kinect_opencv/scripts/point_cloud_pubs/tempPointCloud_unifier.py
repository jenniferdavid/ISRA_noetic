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

def append_pointCloud(pointCloud1, pointCloud2):

    appended_pointCloud = PointCloud()
    #filling pointcloud header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = '/odom' #explains what frame the point cloud data is in
    appended_pointCloud.header = header

    for i in range(len(pointCloud2.points)):
        pointCloud1.points.append(pointCloud2.points[i])
    
    appended_pointCloud.points = pointCloud1.points

    return(appended_pointCloud)


        
def process_pointClouds(front_point_sub, back_point_sub, left_point_sub, right_point_sub):
    ''' 
        This function is the callback function that is run when data is received on the 
        point cloud topics. It takes point clouds as input. It then processes
        this information and publishes a unified point cloud.
    '''

    
    try:      
        print("running")
        pointCloudOut = PointCloud()
    
        pointCloudOut = append_pointCloud(front_point_sub, back_point_sub)
        pointCloudOut = append_pointCloud(pointCloudOut, left_point_sub)
        pointCloudOut = append_pointCloud(pointCloudOut, right_point_sub)
        

        pointcloud_publisher.publish(pointCloudOut)

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
    front_point_sub = message_filters.Subscriber("/temperature_front_pointcloud_topic", PointCloud)
    back_point_sub = message_filters.Subscriber("/temperature_back_pointcloud_topic", PointCloud) ## TODO: CHANGE THIS TO BE RADIATION AGAIN
    left_point_sub = message_filters.Subscriber("/temperature_left_pointcloud_topic", PointCloud)
    right_point_sub = message_filters.Subscriber("/temperature_right_pointcloud_topic", PointCloud)

    
    #Ensure data is time synchronised and send to the call back function
    ts = message_filters.TimeSynchronizer([front_point_sub, back_point_sub, left_point_sub, right_point_sub], 10)
    ts.registerCallback(process_pointClouds)
    rospy.spin()

if __name__ == '__main__':
    try:
        pointcloud_publisher = rospy.Publisher("/unified_pointcloud_topic", PointCloud, queue_size = 10)
        start_node()
    except rospy.ROSInterruptException:
        pass