#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from math import atan2

x = 0.0
y = 0.0
theta = 0.0


def newPosition(msg):
    # global x
    # global y
    # global theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    print(x, y, theta)





if __name__ == "__main__":
    rospy.init_node("get_position", anonymous=True)

    sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, newPosition)

    rospy.spin()
