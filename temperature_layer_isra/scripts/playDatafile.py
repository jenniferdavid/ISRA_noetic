#!/usr/bin/env python

import rospy
import rospkg # Used to find path to data file
import numpy as np # Used for simplified data file reading
import geometry_msgs.msg as gmsg
import sensor_msgs.msg as senmsg
import tf2_ros as tf2



class filePublisher(object):
    """docstring for filePublisher"""
    def __init__(self):
        rospy.init_node("temperatureFromFile") # Init node called "temperatureFromFile"
        remperatureTopic = rospy.get_param("~temperatureName","temperatureTopic") # Name of the topic containing temperature data

	self.temperaturePublisher = rospy.Publisher(temperatureTopic, senmsg.Image, queue_size=2) # Publisher of image messages carrying temperature information
        self.globalFrame = rospy.get_param("~globalFrame","map") # Set the frame in which the pointcloud should be referenced to
        self.childFrame = rospy.get_param("~childFrame","base_link") # Set the frame in which the pointcloud should be referenced to

	# Load data file
	# Data file format: x, y, z, counts
	package_path = rospkg.RosPack().get_path('temperature_layer') 
	data_file = '/data/temperature_sample.dat'
	self.tempData = np.loadtxt(package_path + data_file, delimiter=',') # Data file is comma seperated
        nSamples = self.tempData.shape[0]

	# temperature intensity scaling factor
	# Max value of temperature data is 38, therefore try to scale this to 254 of green channel
	# Whilst remaining integer and linear
	# Why 254 and not 255? LETHAL_OBSTACLE in costmap = 254
	self.intensity_scale_factor =  np.floor(254.0/max(self.tempData[:,3]))

        self.seq = 0

        rate = rospy.Rate(2) #Rate in Hz

        while not rospy.is_shutdown():
            if self.seq < nSamples:
                self.publishdata(self.tempData[self.seq,:])
                rate.sleep()
            else:
                rate.sleep()

    def publishdata(self, data):
        time = rospy.Time.now()
	# Update TF to match sensor location in x, y, z
        self.setTF(data[0], data[1], data[2], time)
	
	# Create blank image message
        message = senmsg.Image()
	# Populate header information
	message.header.seq = self.seq
	message.header.stamp = time
        message.header.frame_id = self.childFrame
	# Set encoding type
	message.encoding = 'rgb8'
	# Single pixel camera
        message.height = 1
	message.width = 1
	
	
	# Set value of pixel to match temperature value
	# Green channel = int( intensity modifier * temperature counts data)
        message.data = [0, int(self.intensity_scale_factor*data[3]), 0] # Red, GREEN, blue channels, only green is populated
        self.temperaturePublisher.publish(message)
        self.seq += 1
	

    def setTF(self, x, y, z, time):
    	br = tf2.TransformBroadcaster()
        t = gmsg.TransformStamped()

        t.header.stamp = time
        t.header.frame_id = self.globalFrame
        t.child_frame_id = self.childFrame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
	
	# Assume no rotation
    	t.transform.rotation.x = 0.0
    	t.transform.rotation.y = 0.0
    	t.transform.rotation.z = 0.0
    	t.transform.rotation.w = 1.0

        br.sendTransform(t)


if __name__ == "__main__":

    try:
        print("Starting temperature data player")
        s = filePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass
