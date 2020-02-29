#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tello_driver.msg import TelloStatus
import threading
import sys
import tf.transformations as tftr
from numpy import *

lock = threading.Lock()

# DEFINES

FRAC_PART = 4

EPSILON = 0.1
V_MAX   = 0.4 # m/s
W_MAX   = 0.5 # rad/s


class Tello:

    def __init__(self):
        # Creates a node with name 'tello_controller' and make sure it is a
        # unique node
        rospy.init_node('tello_controller', anonymous=True)

        self.takeoff_publisher = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_publisher = rospy.Publisher('/tello/land', Empty, queue_size=1)

        self.velocity_publisher = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=5)

        self.status_subscriber = rospy.Subscriber('/tello/status', TelloStatus, self.get_status)
        
        self.status = None

        self.odom_subscriber = rospy.Subscriber('/tello/odom', Odometry, self.update_odom)

        self.x   = None
        self.y   = None
        self.z   = None
        self.q   = None
        self.rpy = None

        self.start = Point()

        self.rate = rospy.Rate(60)

    def get_status(self, data):
        # Dron status callback function
        lock.acquire()

        self.status = data

        lock.release()

    def set_start(self):
        self.start.x = self.x
        self.start.y = self.y
        self.start.z = self.z

    def update_odom(self, data):
        # Odometry callback function
        lock.acquire()

        self.x = round(data.pose.pose.position.x, FRAC_PART)
        self.y = round(data.pose.pose.position.y, FRAC_PART)
        self.z = round(data.pose.pose.position.z, FRAC_PART)

        self.q = data.pose.pose.orientation
        self.rpy = tftr.euler_from_quaternion((self.q.x, self.q.y, self.q.z, self.q.w))  # roll pitch yaw

        lock.release()

    def linear_distance(self, goal_point):
        return sqrt((goal_point.x - self.x)**2 +
                    (goal_point.y - self.y)**2 +
                    (goal_point.z - self.z)**2 )

    def angular_distance(self, goal_pose):
        # in oXY
        return sqrt(pow((goal_pose.theta - self.pose.theta), 2))

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=30):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta) 

    def saturation(self, vel_raw):
        # v_x
        if vel_raw.linear.x > V_MAX:
            vel_raw.linear.x = V_MAX
        elif vel_raw.linear.x < -V_MAX:
            vel_raw.linear.x = -V_MAX
        # v_y
        if vel_raw.linear.y > V_MAX:
            vel_raw.linear.y = V_MAX
        elif vel_raw.linear.y < -V_MAX:
            vel_raw.linear.y = -V_MAX
        # v_z
        if vel_raw.linear.z > V_MAX:
            vel_raw.linear.z = V_MAX
        elif vel_raw.linear.z < -V_MAX:
            vel_raw.linear.z = -V_MAX
        # w_x
        if vel_raw.angular.x > W_MAX:
            vel_raw.angular.x = W_MAX
        elif vel_raw.angular.x < -W_MAX:
            vel_raw.angular.x = -W_MAX
        # w_y
        if vel_raw.angular.y > W_MAX:
            vel_raw.angular.y = W_MAX
        elif vel_raw.angular.y < -W_MAX:
            vel_raw.angular.y = -W_MAX
        # w_z
        if vel_raw.angular.z > W_MAX:
            vel_raw.angular.z = W_MAX
        elif vel_raw.angular.z < -W_MAX:
            vel_raw.angular.z = -W_MAX

        return vel_raw

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

        self.velocity_publisher.publish( self.saturation(vel_msg) )

    def go_to_point(self, goal_point):
        k_p = 3

        err = self.linear_distance(goal_point)
        while abs(err) > EPSILON:
            v_x = k_p * (goal_point.x - self.x)
            v_y = k_p * (goal_point.y - self.y)
            v_z = k_p * (goal_point.z - self.z)

            self.set_velocity(v_x, v_y, v_z)


            self.rate.sleep()

            err = self.linear_distance(goal_point)
            print 'Err: {0}, vx : {1}, vy : {2}'.format(err, v_x, v_y)

        self.set_velocity()

if __name__ == '__main__':
    try:
        drone = Tello()
        rospy.Rate(1).sleep() # Setiing up a subscriber may take a while ...

        print 'Taking off ...'
        drone.take_off()
        rospy.sleep(7)
        print 'Start position : [{0}, {1}, {2}]'.format(drone.start.x, drone.start.y, drone.start.z)

        print '\n Status : {} \n'.format(drone.status)

        a = Point(drone.start.x, drone.start.y, drone.start.z)
        a.x += 0.3
        a.y += 0.3
        a.z -= 0.2

        print 'Going to point [{0}, {1}, {2}] ...'.format(a.x, a.y, a.z)
        drone.go_to_point(a)
        rospy.sleep(5)

        print 'Landing ...'
        drone.land()
        rospy.sleep(1)

        print('killing controller ...') 
    #rospy.spin()
    except rospy.ROSInterruptException:
        pass