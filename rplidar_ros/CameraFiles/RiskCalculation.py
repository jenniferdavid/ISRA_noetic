#!/usr/bin/env python


import rospy
import rospkg
import os
import json
import numpy as np
import random
import time
import sys
import rosservice
import cv2
from CameraSetup import CameraSetup
from CameraSetup import ImageRecognition
from CameraRiskDetection import RiskCalculation
from obstacle_distance import LidarReading
from std_msgs.msg import Float32MultiArray




if __name__ == '__main__':
    rospy.init_node('Risk_analysis', anonymous=True)
    pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
    pub_get_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)
    result = Float32MultiArray()
    get_action = Float32MultiArray()

    camera = CameraSetup()
    dnnRecognition = ImageRecognition()
    riskAnalysis = RiskCalculation()
    lidar = LidarReading()

    rospy.loginfo("Everything on, starting loop: ")
    flag = True
    try:
        while flag == True:
            DempsterNumber = 4
            riskCam = []
            riskLidar = []
            percentageCam = []
            percentageLidar = []

            while DempsterNumber > 0:  ##Calculation for Belief and Plausibility, Creating a Table of size 4

                color_image, colorized_depth, aligned_depth_frame = camera.getFrames()

                shrinkImage, shrinkDepth = camera.imagePreprocessing(color_image, colorized_depth, dnnRecognition.expectedSize)

                output_color, listOfValues = dnnRecognition.classification(shrinkImage)
                dist = dnnRecognition.getDepthData(colorized_depth, aligned_depth_frame, camera, listOfValues)

                data = lidar.readScanners()
                angle, min_obstacle = lidar.getObstacles(data)

                ##only for tshoot####
                # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                # cv2.imshow('RealSense', shrinkImage)
                # print("Detected a {0} {1:.3} meters away.".format(listOfValues[4], dist))
                #############

                camRisk = riskAnalysis.objectRisk(listOfValues[4])
                lidarRisk = riskAnalysis.distanceRisk(min_obstacle)
                # print(lidarRisk)


                riskCamT, percentageCamT = riskAnalysis.riskCatalog(camRisk)
                riskLidarT, percentageLidarT = riskAnalysis.riskCatalog(lidarRisk)
                riskCam.append(riskCamT)
                riskLidar.append(riskLidarT)
                percentageCam.append(percentageCamT)
                percentageLidar.append(percentageLidarT)

                DempsterNumber-=1
            # print("riskCam: ", riskCam, ", riskLid: ", riskLidar)
            risk, belief, pl = riskAnalysis.SensorFusion(riskCam, percentageCam, riskLidar, percentageLidar)
            rospy.loginfo("Risk detected: %s with: %d %% \n Object Detected: %s, Distance: %d", risk, belief*100, listOfValues[4], lidarRisk)


            ############Prospect Theory

            vx, px = riskAnalysis.prospectTheory(4 - risk, belief) ## the more the risk, the less gain it should have
            vy, py = riskAnalysis.prospectTheory(4, 1-belief) ##VY will always be 4 simulating a good choice

            Vx1 = vx*px + vy*py
            ####This second value represent complete uncertainty that must be modified in the future
            vx, px = riskAnalysis.prospectTheory(3, 0.5)
            vy, py = riskAnalysis.prospectTheory(2, 0.5)
            Vx2 = vx*px + vy*py

            if(Vx1 >= Vx2):
                rospy.loginfo("Continue over this path")
            else:
                rospy.loginfo("Change over the uncertainty Path")





            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                flag = False
                # camera.stopProcess()
                break

            # time.sleep(1)

    finally:
        # Stop streaming
        camera.stopProcess()
        print("finished")