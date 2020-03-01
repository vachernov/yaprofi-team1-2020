#!/usr/bin/env python
# coding: utf-8
"""
Module contains tools for trajectory generation
"""
import rospy
from numpy import sqrt, sin, cos, arctan2, pi, trunc, finfo, float32

from geometry_msgs.msg import Pose2D, Twist, Vector3
from tello_driver.msg import CartesianTrajectory
from tello_driver.srv import SetTrajectoryParameters


class RobotinoTrajectoryGenerator:

    def __init__(self):
        self.RATE = rospy.get_param('/rate', 5)

        self.dt = 1. / self.RATE

        # trajectory parameters (initial)
        self.laps = 4   # quantity laps
        self.velocity = 0.15
        self.A_x, self.A_y = 1., 0.5
        self.phi = -pi / 2.

        self.tf = 1  # time for lap. Just for c
        self.omega_x = 2 * pi / self.tf
        self.omega_y = 2 * self.omega_x     # frequencies for 8-like-trajectory
        self.N = self.tf / self.dt  # quantity of a trajectory points

        self.is_update = True   # update trajectory parameters

        self.trajectory_pub = rospy.Publisher('/tello/trajectory', CartesianTrajectory, queue_size=-1)
        self.trajectrory_parameters_srv = None
        try:
            self.trajectrory_parameters_srv = rospy.Service('/set_trajectory_parameters', SetTrajectoryParameters, self.trajectrory_parameters_handler)
        except rospy.service.ServiceException as se:
            # it's ok. Service already registered
            pass

    def close(self):
        self.trajectory_pub.unregister()
        self.trajectrory_parameters_srv.shutdown()

    def trajectrory_parameters_handler(self, req):
        """
        Sets trajectory parameters
        :param req: CartesianTrajectory parameters. See msg folder in the bac_task package
        """
        self.laps = req.laps
        self.velocity = req.velocity
        self.A_x, self.A_y = req.A_x, req.A_y
        self.phi = req.phi

        rospy.loginfo('Service called. Trajectory parameters updated!')
        self.is_update = True
        return [True, self.get_trajectory_length()]

    def get_trajectory_length(self):
        l = 0.  # trajectory length
        old_xi, old_yi = 0., 0.
        ti = 0.
        while ti <= self.tf:    # for one lap
            xi = self.A_x * cos(self.omega_x * ti + self.phi)
            yi = self.A_y * sin(self.omega_y * ti)
            dl = sqrt((xi - old_xi)**2 + (yi - old_yi)**2)
            old_xi, old_yi = xi, yi
            l += dl
            ti += self.dt
        return l

    def make_trajectory_eight(self):
        """
        :return: message with cartesian tounrajectory
        """
        l = self.get_trajectory_length()
        self.tf = l / self.velocity

        self.omega_x = 2 * pi / self.tf
        self.omega_y = 2 * self.omega_x     # frequencies for 8-like-trajectory
        self.N = self.tf / self.dt  # quantity of a trajectory points

        trajectory = CartesianTrajectory()
        trajectory.poses = []
        trajectory.velocities = []
        trajectory.time_from_start = []
        trajectory.header.frame_id = 'world'

        old_xi, old_yi = 0., 0.
        ti = 0.
        while ti <= self.tf * self.laps:
            if ti == 0:  # initial conditions for orientation
                old_xi = self.A_x * cos(self.omega_x * self.dt * (self.N - 1) + self.phi)
                old_yi = self.A_y * sin(self.omega_y * self.dt * (self.N - 1))

            # compute trajectory point and orientation
            xi = self.A_x * cos(self.omega_x * ti + self.phi)
            yi = self.A_y * sin(self.omega_y * ti)
            theta = arctan2(yi - old_yi, xi - old_xi)
            old_xi, old_yi = xi, yi

            # compute the derivative of trajectory point and orientation
            dot_xi = -self.A_x * self.omega_x * sin(self.omega_x * ti + self.phi)
            dot_yi = self.A_y * self.omega_y * cos(self.omega_y * ti)

            if abs(xi) < finfo(float).eps:
                xi = xi + finfo(float).eps
            if (yi / xi)**2 == 1.:
                yi = yi + finfo(float).eps
            dot_theta = 1. / (1 + (yi / xi)**2)

            # add the point to message
            trajectory.poses.append(Pose2D(xi, yi, theta))
            trajectory.velocities.append(Twist(Vector3(dot_xi, dot_yi, 0.), Vector3(0., 0., dot_theta)))
            trajectory.time_from_start.append(rospy.Duration(ti))

            ti += self.dt

        rospy.loginfo('Trajectory for Robotino2 cooked!')
        rospy.loginfo('laps = {}'.format(self.laps))
        rospy.loginfo('velocity = {}'.format(self.velocity))
        rospy.loginfo('lap duration = {}'.format(ti))
        rospy.loginfo('length (approx.) = {}'.format(l))
        rospy.loginfo('points (quantity) = {}'.format(self.N))

        return trajectory

    def spin(self):
        trajectory = None
        while not rospy.is_shutdown():
            if self.is_update:
                trajectory = self.make_trajectory_eight()
                self.is_update = False

            if trajectory:
                trajectory.header.stamp = rospy.get_rostime()
                self.trajectory_pub.publish(trajectory)
            rospy.sleep(0.02)


if __name__ == '__main__':
    rospy.init_node('robotino_trajectory_generator_node')
    robotino_tg = None
    while not rospy.is_shutdown():
        try:
            robotino_tg = RobotinoTrajectoryGenerator()
            robotino_tg.spin()
        except rospy.ROSInterruptException as e:
            robotino_tg.close()
            del robotino_tg
            print('Restart robotino_trajectory_generator_node with new ros-sim-time')
