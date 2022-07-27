#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def showImage(img):#, red_pixels_x, red_pixels_y):
    '''
        Takes an image as an argument and then displays that image.
    ''' 
    #Display image
    cv2.imshow('image', img)
    cv2.waitKey(1)

    #cv2.circle(img, circleCenter , 1, (0,255,0), thickness=1, lineType=8, shift=0)
    # ^^^ Allows circles to be drawn on image if necessary for investigating certain pixels and visualising easily

def process_image(msg):
    try:
        # convert sensor_msgs/Image to OpenCV Image
        bridge = CvBridge()
        drawImg = bridge.imgmsg_to_cv2(msg, "bgr8")

        #Creates a mask using HSV and a colour threshold
        img_hsv=cv2.cvtColor(drawImg, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100,150,0])
        upper_blue = np.array([140,255,255])
        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        #Determines where the mask is equal to 0  (i.e. where the red pixels are)
        blue_pixels_x = np.where(mask != 0)[0]
        blue_pixels_y = np.where(mask != 0)[1]

        # show results of the mask operation to test
        showImage(mask)

    except Exception as err:
            print err

def start_node():
    '''
        This function is run when the program starts. It initialises the node and 
        subscribes to the relevant topics. It then spins until the node is terminated.
    '''

    #Initialise this node
    rospy.init_node('detect_image')
    rospy.loginfo('image node started')

    #Subscribe to the camera
    rospy.Subscriber("/temperature_front/color/image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass