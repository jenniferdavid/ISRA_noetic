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
from geometry_msgs.msg import Point32, PoseArray
from rospy_tutorials.msg import Floats
from kinect_opencv.msg import StampedArray

def get_point_data(pCloud, u, v):
    ''' 
        This Function takes a point cloud and pixel location and determines the 
        x, y, z of the point that is associated with that pixel in the kinect's frame.
        It then converts the axis to the correct axis as the camera and ROS frames differ.
    '''

    # Gets the index of the point in the point cloud
    index = v*pCloud.row_step + u*pCloud.point_step

    #Unpacks the point associated with the given index
    (x,y,z) = struct.unpack_from('fff', pCloud.data, offset=index)

  
    #correct the axis as they are different in ROS than from camera's
    X = z
    Y = -x
    Z = -y 

    #Puts the position into a list to be returned and returns it
    position = [X,Y,Z]
    return(position)

def convert_to_world_frame(position, trans_mat):
    ''' 
        This function takes a position and a transformation matrix and applies the
        transformation to the position. In this code it is used to transform the 
        kinect frame to the world frame. The transformed position is returned.
    '''
    # Creates an array from the position so that numpy can be used
    pos_kinect_frame = np.array([position[0],position[1],position[2],1])

    #Applies the transform to the position using matrix multiplication
    pos_world_frame = np.dot(trans_mat,pos_kinect_frame.transpose())[0:3].transpose()

    return(pos_world_frame)

        
def process_camera(image, camera_info, depth):
    ''' 
        This function is the callback function that is run when data is received on the 
        image, point cloud and camera_info topics. It takes an image and point cloud as input,
        as well as the info relating to the kinect (or other camera) sensor. It then processes
        this information and publishes a point cloud of only the pixels of a given colour 
        (in this case red).
    '''
    try:
        # convert sensor_msgs/Image to OpenCV Image for processing
        bridge = CvBridge()
        colour_image = bridge.imgmsg_to_cv2(image, "bgr8")

        #listens to the transform from the kinect frame to the odom frame
        listener = tf.TransformListener()
        listener.waitForTransform("/odom","/radiation_front_link", rospy.Time(0), rospy.Duration(3.0))
        (trans,rot) = listener.lookupTransform("/odom","/radiation_front_link", rospy.Time(0))

        #Creates a transformer object and determines the transformation matrix from the trans
        # and rot (quaternion) attained from listener
        transformer = tf.TransformerROS()
        trans_mat = transformer.fromTranslationRotation(trans,rot)

        #Creates a mask using HSV and a colour threshold
        # img_hsv=cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
        # lower_red = np.array([0,100,100])
        # upper_red = np.array([10,255,255])
        # mask = cv2.inRange(img_hsv, lower_red, upper_red)

        # #Determines where the mask is equal to 0  (i.e. where the red pixels are)
        # red_pixels_x = np.where(mask != 0)[0]
        # red_pixels_y = np.where(mask != 0)[1]

        pos_world_frame = []
        pixel = []

        for u in range(120):
            for v in range(120):
                # get the position in the kinect frame from point cloud data
                pos_kinect_frame = get_point_data(depth, u, v)
                

                # convert the position into the world frame with tf.transformer
                pos_world_frame.append(convert_to_world_frame(pos_kinect_frame, trans_mat)) #= convert_to_world_frame(pos_kinect_frame, trans_mat)

                pixel.append(colour_image[u][v])
       
        

        # # Loops through the red pixels and finds their position in the world frame, with factor for speeding process up by sparsifying 
        # if len(red_pixels_x) > 0:
        #     # sparsify the number of pixels being looked through, works for dense images, 
        #     speed = 200 # determines how much faster (and sparser) the matrix should be set to 0 for all elements to be looked through
        #     i=0
        #     pos_world_frame = []
        #     while i < len(red_pixels_x):
        #         #gets pixel location
        #         u = red_pixels_y[i]
        #         v = red_pixels_x[i]

        #         # get the position in the kinect frame from point cloud data
        #         pos_kinect_frame = get_point_data(depth, u, v)

        #         # convert the position into the world frame with tf.transformer
        #         pos_world_frame.append(convert_to_world_frame(pos_kinect_frame, trans_mat)) #= convert_to_world_frame(pos_kinect_frame, trans_mat)

        #         #uses the 'speed' parameter to take multiples of speed from pixel array to more sparsly populate the position in the world frame
        #         if i<len(red_pixels_x)-speed:
        #             i=i+speed
        #         else:
        #             i=i+1
        #             break
                
        # else:
        #     #Let's user no when no red pixels have been detected
        #     print("No Red Pixels")

        point_cloud = []

        for i in range(len(pos_world_frame)):
            if pixel[i][1] > 10:
                point_cloud.append(pos_world_frame[i][0])
                point_cloud.append(pos_world_frame[i][1])
                point_cloud.append(pos_world_frame[i][2])
                point_cloud.append(pixel[i][1])
        

        # This loop part of the code populates a pointcloud() ROS message and publishes it
    
        #pointcloud_publisher.publish(point_cloud)
        pointCloud = StampedArray()

        #filling PoseArray header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/odom' #explains what frame the point cloud data is in
        pointCloud.header = header
        pointCloud.array = point_cloud

        pointcloud_publisher.publish(pointCloud)
            
        
        




    except Exception as err:
            print err

def start_node():
    '''
        This function is run when the program starts, after declaration of the publisher. 
        It initialises the node and subscribes to the relevant topics. It then spins until
        the node is terminated.
    '''

    #Initialise this node
    rospy.init_node('radiation_pixel_processor')
    rospy.loginfo('radiation_pixel_processor node started')
    
    #Subscribe to the camera, point cloud and camera info topics
    image_sub = message_filters.Subscriber("/radiation_front/color/image_raw", Image)
    info_sub = message_filters.Subscriber("/radiation_front/color/camera_info", CameraInfo)
    depth_sub = message_filters.Subscriber("radiation_front/depth/points", PointCloud2)
    
    #Ensure data is time synchronised and send to the call back function
    ts = message_filters.TimeSynchronizer([image_sub, info_sub, depth_sub], 10)
    ts.registerCallback(process_camera)
    rospy.spin()

if __name__ == '__main__':
    try:
        pointcloud_publisher = rospy.Publisher("/radiation_pixel_topic", StampedArray, queue_size = 10)
        start_node()
    except rospy.ROSInterruptException:
        pass