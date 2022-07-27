#! /usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist

x = 0.0
y = 0.0
theta = 0.0

class Calibration:

    def __init__(self):

        # Creates a node with name 'speed_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node("speed_controller", anonymous=True)
        # Publisher which will publish to the topic '/cmd_vel'.

        self.pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.newPosition)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # A subscriber to the topic '/amcl_pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose = PoseWithCovarianceStamped()
        self.r = rospy.Rate(4)

    def newPosition(self, msg):
        # Add these lines if required to use x and y
        # global x
        # global y
        global theta
        # x = msg.pose.pose.position.x
        # y = msg.pose.pose.position.y


        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        # print(x, y, theta)



    def calibration(self):
        speed = Twist()  ### Speed has to be more than 0.5 in order for the robot to move

        angle_to_goal_pos = math.pi
        angle_to_goal_neg = 0
        positive_flag = True
        end_flag = False

        while not end_flag:

            speed.angular.z = 0.7
            self.velocity_publisher.publish(speed)
            self.r.sleep()
            if positive_flag:
                if not (abs(angle_to_goal_pos - theta) > 0.2):
                    positive_flag = False
            else:
                if not (abs(angle_to_goal_neg - theta) > 0.2):
                    end_flag = True


        speed.angular.z = 0.0
        self.velocity_publisher.publish(speed)
        print("Calibrated!")
        rospy.spin



if __name__ == "__main__":
    try:
        x = Calibration()
        x.calibration()
    except rospy.ROSInterruptException:
        pass





