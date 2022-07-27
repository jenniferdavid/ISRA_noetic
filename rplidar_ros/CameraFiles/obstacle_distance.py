#!/usr/bin/env python


import rospy
import rospkg
import os
import numpy as np
import random
import time
import sys

# from environment_stage import Env

from sensor_msgs.msg import LaserScan

class LidarReading:

# def callback(msg):
#     data360 = None
#
#     data360 = msg
#     # print("min: ", max(dataF.ranges))
#
#     obstacle_angle, min_range = getObstacles(data360)
#     risk_level = riskCalculation(min_range)
#     rospy.loginfo("Obstacle Distance: %f, obstacle Angle: %d, risk: %d ", min_range, obstacle_angle, risk_level)
#     # for i in range(len(min_range)): ##we need to evaluate more angles
#     #     rospy.loginfo("Obstacle Distance: %f, obstacle Angle: %d ", min_range[i], obstacle_angle[i])

    def readScanners(self):
        data = None

        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass


        return data

    def getObstacles(self, data360):
        scan_range = data360.ranges

        possibleObstacles = []
        obstaclesAngle = []
        for i in range(len(scan_range)):
            if scan_range[i]>0.15 and scan_range[i]<2:
                possibleObstacles.append(scan_range[i])
                obstaclesAngle.append(i)

        min_obstacle = min(possibleObstacles)
        angle = obstaclesAngle[np.argmin(possibleObstacles)]


        return angle, min_obstacle
#
    def riskCalculation(self, obstacleDistance):
        maxDist = 2
        minDist = 0.2
        step = (maxDist - minDist)/100
        risk = (maxDist - obstacleDistance)/step

        return risk


if __name__ == '__main__':
    rospy.init_node('rp_lidar_obstacles', anonymous=True)
    rospy.loginfo("Initializing node reading Lidar")
    lidar = LidarReading()

    try:

        while True:

            data = lidar.readScanners()

            angle, min_obstacle = lidar.getObstacles(data)

            risk = lidar.riskCalculation(min_obstacle)
            print("Risk: ", risk)

    except KeyboardInterrupt:
        print("Finished")

