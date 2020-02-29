#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import threading
import sys
import tf.transformations as tftr
from numpy import *

lock = threading.Lock()

# DEFINES

FRAC_PART = 4

class Tello:

    def __init__(self):
        # Creates a node with name 'tello_controller' and make sure it is a
        # unique node
        rospy.init_node('tello_controller', anonymous=True)

        # Publisher which will publish to the topic '/tello/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=5)

        self.odom_subscriber = rospy.Subscriber('/tello/odom', Odometry, self.update_odom)

        self.x   = None
        self.y   = None
        self.z   = None
        self.q   = None
        self.rpy = None

        self.rate = rospy.Rate(60)

    def update_odom(self, data):
        # Odometry callback function
        lock.acquire()

        self.odom = data

        self.x = round(self.odom.pose.pose.position.x, FRAC_PART)
        self.y = round(self.odom.pose.pose.position.y, FRAC_PART)
        self.z = round(self.odom.pose.pose.position.z, FRAC_PART)

        self.q = self.odometry.pose.pose.orientation
        self.rpy = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))  # roll pitch yaw

        lock.release()

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def angular_distance(self, goal_pose):
        return sqrt(pow((goal_pose.theta - self.pose.theta), 2))

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=30):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)    

    def set_velocity(self, v_x, v_y, v_z, w_x, w_y, w_y):

        goal_pose = Pose()

        vel_msg.linear.x = v_x
        vel_msg.linear.y = v_y
        vel_msg.linear.z = v_z

        vel_msg.angular.x = w_x
        vel_msg.angular.y = w_y
        vel_msg.angular.z = w_z

        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        drone = Tello()
        rospy.Rate(1).sleep() # Setiing up a subscriber may take a while ...
        print('killing controller ...') 
    #rospy.spin()
    except rospy.ROSInterruptException:
        pass