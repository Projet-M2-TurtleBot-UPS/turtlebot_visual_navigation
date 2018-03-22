# module for classes

# title           : kalman_class.py module for classes of ekf_module.py
# description     : This module contains all functions of ekf_module.py
# author          : Salah Eddine Ghamri
# date            : 17-03-2018
# version         : 0.2
# usage           : /
# notes           :
# python_version  : 2.6.7
# Platforme       : Ubuntu
# =====================================================================

import numpy as np
import rospy
from scipy.linalg import block_diag
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point, Quaternion


class kalman_class():

    def __init__(self, x, P):

        self.previous_x = np.array(x)
        self.previous_P = np.diag(P)
        self.time = 0
        self.estimated_x = None
        self.estimated_P = None
        self.predicted_x = 0 * self.previous_x
        self.predicted_P = 0 * self.previous_P

        # node initialisation
        rospy.init_node("ekf_module")
        Rate = rospy.Rate(100)

        self.C = np.array([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0],
                           [0, 0, 1, 0, 0],
                           [0, 0, 0, 1, 0],
                           [0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1],
                           [1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0],
                           [0, 0, 1, 0, 0]])

    def predict(self, T, sigma_v, sigma_omega):

        self.predicted_x[2, 0] = self.previous_x[2, 0] + self.previous_x[4, 0] * T
        # norm angle [-pi pi]
        self.predicted_x[2, 0] = ((self.predicted_x[2, 0] + np.pi) % (2*np.pi)) - np.pi

        """
        # angle normalisation between pi and -pi
        if self.x[2, 0] >= np.pi:
        self.x[2, 0] = -np.pi  + (self.x[2, 0] - np.pi)
        """
        # -------------------------------------------------------------------
        self.predicted_x[0, 0] = self.previous_x[0, 0] + self.previous_x[3, 0] * T * np.cos(self.predicted_x[2, 0])
        self.predicted_x[1, 0] = self.previous_x[1, 0] + self.previous_x[3, 0] * T * np.sin(self.predicted_x[2, 0])

        self.predicted_x[3, 0] = self.previous_x[3, 0]
        self.predicted_x[4, 0] = self.previous_x[4, 0]

        # d_f is the jacobian of f for u = [v, omega]'

        ang = T*self.previous_x[4, 0] + self.previous_x[2, 0]
        ang = ((ang + np.pi) % (2*np.pi))-np.pi

        d_f = np.array([[T*np.cos(ang), (-(T**2)*self.previous_x[3, 0]*np.sin(ang))],
                        [T*np.sin(ang), (T**2)*self.previous_x[3, 0]*np.cos(ang)],
                        [0, T],
                        [1, 0],
                        [0, 1]])

        # d_f_prime is the jacobian of f for x
        d_f_prime = np.array([[1, 0, -T*self.previous_x[3, 0]*np.sin(ang), T*np.cos(ang), -(T**2)*self.previous_x[3, 0]*np.sin(ang)],
                              [0, 1, T*self.previous_x[3, 0]*np.cos(ang), T*np.sin(ang), (T**2)*self.previous_x[3, 0]*np.cos(ang)],
                              [0, 0, 1, 0, T],
                              [0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 1]])

        var_Q = np.array([[sigma_v**2, 0], [0, sigma_omega**2]])
        Q = np.dot(d_f, np.dot(var_Q, d_f.T))
        self.predicted_P = np.dot(d_f_prime, np.dot(self.previous_P, d_f_prime.T)) + Q
        # print("i predicted")

    def Kalman_gain(self, odom_covariance, imu_covariance, vo_covariance):
        # covariances here are extracted from /odom, /vo and /Imu
        # measurement in forme z = Cx + v , v is noise
        self.R = np.array(block_diag(odom_covariance, imu_covariance, vo_covariance))
        print(self.R)
        self.S = np.dot(self.C, np.dot(self.predicted_P, self.C.T)) + self.R
        self.K = np.dot(self.predicted_P, np.dot(self.C.T, np.linalg.inv(self.S)))
        # print("i calculated gain")

    def estimate(self, measure):
        # calculates the error between measurement and prediction
        # update x and P
        # returns error
        # changed here vo to odom

        z = np.array([[measure.odom_x], [measure.odom_y],
                      [measure.odom_theta], [measure.odom_v],
                      [measure.imu_theta], [measure.imu_omega],
                      [measure.odom_x], [measure.odom_y], [measure.odom_theta]])
        # print(z)

        # error calcualtion
        error = z - np.dot(self.C, self.predicted_x)

        def nrmlz_angle(error):
            y = error
            y[2, 0] = ((y[2, 0] + np.pi) % (2*np.pi)) - np.pi
            return y

        error = nrmlz_angle(error)
        self.estimated_x = self.predicted_x + np.dot(self.K, error)
        # self.P = self.P - np.dot(K, np.dot(self.S, K.T))
        mat = np.eye(5, dtype=int) - np.dot(self.K, self.C)
        self.estimated_P = np.dot(mat, self.predicted_P)
        # self.P = np.dot(mat, np.dot(self.predicted_P, mat.T)) + np.dot(self.K, np.dot(self.R, self.K.T))
        # self.P = self.predicted_P - np.dot( self.K, np.dot(self.C, self.predicted_P))
        # print("i made estimation")
        # previous update
        self.previous_x = self.estimated_x
        self.previous_P = self.estimated_P
        self.update_velocity()
        return error

    def callback_velocity(self, msg):
        # calculates velocity and updates x
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        v = np.sqrt(vx**2 + vy**2)
        self.previous_x[3, 0] = v
        self.previous_x[4, 0] = omega

    def update_velocity(self):
        # subscriber
        rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.callback_velocity)

    def publish_message(self):
        # publisher
        Pub = rospy.Publisher('/odom_combined', Odometry, queue_size=10)
        msg_odom = Odometry()
        # Publish kalma.x and kalman.P
        x = self.estimated_x[0, 0]
        y = self.estimated_x[1, 0]
        # we don't care for z <-- odometry
        z = 0
        # print("this is predicted", x, y)
        # extracting Quaternion from the homogeneous
        quatern = quaternion_from_euler(0, 0, self.estimated_x[2, 0])
        # Constructing the message
        msg_odom.header.stamp = self.time
        msg_odom.header.frame_id = '/base_foot'
        # msg.header.child_frame_id = child_frame_id
        msg_odom.pose.pose.position = Point(x, y, z)
        msg_odom.pose.pose.orientation = Quaternion(*quatern)
        p = np.diag([self.estimated_P[0, 0], self.estimated_P[1, 1], 10000000, 10000000, 1000000, self.estimated_P[3, 3]])
        msg_odom.pose.covariance = tuple(p.ravel().tolist())
        # publishing the message
        Pub.publish(msg_odom)

    def time_update(self):
        # read time (for message)
        self.time = rospy.Time.now()


class caller():

    def __init__(self):

        self.n = 0
        self.odom_covariance = None
        self.imu_covariance = None
        self.vo_covariance = None
        self.odom_x = None
        self.odom_y = None
        self.odom_theta = None
        self.odom_v = None
        self.imu_theta = None
        self.imu_omega = None
        self.vo_x = None
        self.vo_y = None
        self.vo_theta = None

    def nrmlz_to_2pi(self, angle):
        angle += np.pi
        return angle

    def read_sensors(self):

        # Odometry, Imu, Vo readings and covariances --> caller object.
        rospy.Subscriber('/odom', Odometry, self.callback_odom)

        if self.n == 0:
            rospy.wait_for_message('/odom', Odometry, timeout=1)
        rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, self.callback_imu)

        if self.n == 0:
            rospy.wait_for_message('/mobile_base/sensors/imu_data', Imu, timeout=1)
        rospy.Subscriber('/vo', Odometry, self.callback_vo)
        if self.n == 0:
            rospy.wait_for_message('/vo', Odometry, timeout=1)
        # not first time
        self.n = 1

    def callback_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        # print("this is odom", self.odom_x, self.odom_y)
        # theta is the yaw we need to use tf.transforms quater -> euler
        # the angle the euler_from_quaternion gives is in [-pi, pi]
        quat = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        self.odom_theta = yaw

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.odom_v = np.sqrt(vx**2 + vy**2)
        pose_covariance = msg.pose.covariance
        twist_covariance = msg.twist.covariance
        self.odom_covariance = np.diag([pose_covariance[0],
                                        pose_covariance[7],
                                        pose_covariance[35],
                                        (twist_covariance[0] + twist_covariance[7])])

    def callback_imu(self, msg):
        # message de type Imu
        quat = msg.orientation
        # angle in [-pi, pi]
        self.imu_theta = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2]

        self.imu_omega = msg.angular_velocity.z
        self.imu_covariance = np.diag((msg.orientation_covariance[8],
                                       msg.angular_velocity_covariance[8]))

    def callback_vo(self, msg):
        # same as odometry message
        self.vo_x = msg.pose.pose.position.x
        self.vo_y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        # angle in [-pi, pi]
        self.vo_theta = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2]
        pose_covariance = msg.pose.covariance
        self.vo_covariance = np.diag([pose_covariance[0],
                                     pose_covariance[7],
                                     pose_covariance[35]])
