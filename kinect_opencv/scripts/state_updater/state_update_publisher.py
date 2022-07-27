#!/usr/bin/env python
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


state_publisher = rospy.Publisher("/robot_state_publisher", StampedArray, queue_size = 10)


# Sets the starting values for radiation dose and internal temperature 
robot_rad = 0 # assumes this is the robot's first mission into a nuclear environment 
robot_temp = 20 # assume the robot starts at room temperature

# Sets up the maximum dose variables to see max temp and max rad during operation
max_rad = 0
max_temp = 0

def store_max_values(current_temp, current_rad):
    '''
        Stores the maximum temperature and radiation experienced by the robot in global variables.
    '''

    global max_temp
    global max_rad
    
    if current_temp > max_temp:
        max_temp = current_temp
    
    if current_rad > max_rad:
        max_rad = current_rad


def updateTemperature(temp_data, current_state):
    ''' 
        This function takes the robot's current state and temperature measurement and uses it to update the robot's temperature.
        It outputs the new temperature of the robot.

        Description of how pixel value is change into dose:
        The maximum temperature which the MPO-700 is rated to operate at is 35C. Therefore to scale the pixel values to affect the 
        robot we want to ensure there are some temperatures higher than this. To get the correct order of magnitude is seems plausible to divide
        the pixel value by 10 therefore scaling values from 0 - 25.3C. It then makes sense to double these values to give approximate
        temperature values for a nuclear decommissioning environment. (equivalent to dividing by 5!)

        Description of how temperature state is updated:
        Temperature is not simply cumulative, if the environment is cooler than the robot it will reduce in temperature and if it is hotter the 
        robot will increase in temperature. Additionally, the robot will produce its own internal heat through operature. The change in temperature
        is dependent on the body's specific heat capacity and the heat energy received by Q = mc(dT), where Q is heat, m is mass of object, c is
        specific heat and dT is change in temperature. Using this equation and the assumption that all heat lost by the robot is transferred to
        the environment and vice versa, we get Final temperature, Tf, as:
        Tf = (Mc)(Cc)Tc + (Mh)(Ch)Th/ (Mc)(Cc) + (Mh)(Ch)

        where Mc/h is the cold/hot items' mass Ch/m is the hot/ cold items' specific heat and Tc/h are the hot/ cold items' temperatures

    '''
    # To scale the pixel values to appropriate temperatures we divide by 5 - changed to 2 as the maximum pixel value
    # On the ground seems to be around 100 instead of the anticipated 253
    temperature_measure = temp_data/2


    #Model for how temperature reading match to robot change in temp
    #Robot variables:
    Mc = 120 # mass of the robot in Kg
    Cc = 0.9 # use the specific heat capacity of aluminium
    Tc = current_state # take the current robot's temperature

    #Surroundings variables:
    Mh = 2 # take the mass of air surroudning the robot to be 2Kg
    Ch = 0.718 # this is the specific heat capacity of air

    #ensures that the temperature is always at a minimum of 20C
    #also ensures that the number types are correct by casting Th as float if it uses the camera value (which is sent as numpy.uint64)
    if temperature_measure < 20:
        Th = 20.0

        # equation described above to determine final temperature of the robot
        new_state = ((Mc*Cc*Tc) + (Mh*Ch*Th))/ ((Mc*Cc) + (Mh*Ch))
    else:
        Th = temperature_measure # take the temperature of the environment

        # equation described above to determine final temperature of the robot
        new_state = ((Mc*Cc*Tc) + (Mh*Ch*Th.astype(float)))/ ((Mc*Cc) + (Mh*Ch))



    return(new_state)


def updateRadiation(rad_data, current_state):
    ''' 
        This function takes the robot's current state and radiation measurement and uses it to update the robot's dose.
        It outputs the new dose measurement of the robot.

        Description of how pixel value is change into dose:
        From Thesis titled "THE RADIATION TOLERANCE AND DEVELOPMENT OF ROBOTIC PLATFORMS FOR NUCLEAR DECOMMISSIONING" the average dose rate
        for a robot during nuclear decommissioning tasks is 1.4mGy/s. The maximum dose a semi conductor can withstand is 1kGy
        We therefore want to scale the maximum value of dose rate to be approximately 2/2.5mGy/s and adjust the upper threshold so that the robot
        does not exceed a dose of 1Gy or 1000mGy (this is because we are assuming that a single mission may be carried out 1000 times in a year, so cumulative 
        dose is considered)

        Description of how dose state is updated:
        Dose is simply cumulative, so in order to calculate new dose of the robot, we simply add the measured dose to the robot's current state
    '''

    # To achieve above, it is simple enough to divide the pixel reading by 100 to make it into mGy i.e. a pixel of value 254 would be 2.54mGy
    #rad_data.astype(float)

    dose_measure = rad_data.astype(float)/100.0
    

    # The robot dose can be calculated by simply concatentating previous doses
    new_state = current_state + dose_measure


    return(new_state)


        
def process_data(image):
    ''' 
        This function is the callback function that is run when data is received on the 
        image topic that represents the radiation and temperature sensors. It takes the data and updates the states
        using the update state function.
    '''

    # use the global values for the robots state, so that it can be updated everytime step
    global robot_rad
    global robot_temp
    global max_temp
    global max_rad

    try:
        #covert the image to a matrix so that it can be operated on
        bridge = CvBridge()
        data = bridge.imgmsg_to_cv2(image, "bgr8")

        #for some reason the image is outputted as a three dimensional array, make into a 1d array
        data = data[0][0]

        #get the data associated with radiation and temperature measurements (rgb)
        temperature_data = data[2]
        radiation_dose_data = data[1]
        
        #update the robot's current states
        robot_rad = updateRadiation(radiation_dose_data, robot_rad)
        robot_temp = updateTemperature(temperature_data, robot_temp)

        store_max_values(robot_temp, robot_rad)

        print(max_rad, max_temp)


    except Exception as err:
            print err

def start_node():
    '''
        This function is run when the program starts, after declaration of the publisher. 
        It initialises the node and subscribes to the relevant topics. It then spins until
        the node is terminated.
    '''

    #Initialise this node
    rospy.init_node('state_update_publisher')
    rospy.loginfo('state_update_publisher node started')
    
    #Subscribe to the topic that represent temperature and radiation from a camera
    image_sub = rospy.Subscriber("/radiation_front/color/image_raw", Image, process_data)


    # set up the publisher to publish the updated values of the robot state
    #makes sure data is only published once a second (can be slower if necessary but changing r)
    r = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        # Set up the state information
        state = StampedArray()
        #filling PoseArray header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/odom' #explains what frame the point cloud data is in
        state.header = header
        state.array = [robot_rad, robot_temp]
    

        #publish the updated state
        state_publisher.publish(state)
        r.sleep()
    
    #rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass