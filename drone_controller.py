#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
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

        self.takeoff_publisher = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_publisher = rospy.Publisher('/tello/land', Empty, queue_size=1)

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

        self.x = round(data.pose.pose.position.x, FRAC_PART)
        self.y = round(data.pose.pose.position.y, FRAC_PART)
        self.z = round(data.pose.pose.position.z, FRAC_PART)

        self.q = data.pose.pose.orientation
        self.rpy = tftr.euler_from_quaternion((self.q.x, self.q.y, self.q.z, self.q.w))  # roll pitch yaw

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

    def take_off(self):
        msg = Empty()
        self.takeoff_publisher.publish(msg)

    def land(self):
        msg = Empty()
        self.land_publisher.publish(msg)

    def set_velocity(self, v_x=0, v_y=0, v_z=0, w_x=0, w_y=0, w_z=0):

        vel_msg = Twist()

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

        drone.take_off()
        print(10)
        rospy.sleep(5)
        drone.land()

        # rospy.spin()

        print('killing controller ...') 
    #rospy.spin()
    except rospy.ROSInterruptException:
        pass
