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

"""
------prediction-----------
predicted_x(k+1|k) = A(previous_x) previous_x(k|k) + B(previous_x) u(previous_x)
predicted_P(k+1|k) = A(previous_x) previous_P(k|k) A(previous_x)' + G(previous_x)QG(previous_x)'

------kalman---------------
error(k) = z(k) - (C* predicted_x(k|k-1)
S(k) = C * predicted_P(k|k-1) * C' + R(k)
K(k) = predicted_P * C' * S^-1

------estimation------------
estimated_x(k|k) = predicted_x + K(k+1) * error(k+1)
estimated_P(k|k) = (I - K(k+1) * C) predicted_P  
"""

import numpy as np
import rospy
from scipy.linalg import block_diag
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point, Quaternion


class kalman_class():

    def __init__(self, x, P):
	
	self.not_first_time = False
        self.previous_x = np.matrix(x)
        self.previous_P = np.diag(P)
        self.time = 0
        self.estimated_x = self.previous_x
        self.estimated_P = self.previous_P
        self.predicted_x = self.previous_x
        self.predicted_P = self.previous_P

        self.C_full = np.matrix([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0],
                           [0, 0, 1, 0, 0],
                           [0, 0, 0, 1, 0],
                           [0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1],
                           [1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0],
                           [0, 0, 1, 0, 0]])

	self.C_redu = np.matrix([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0],
                           [0, 0, 1, 0, 0],
                           [0, 0, 0, 1, 0],
                           [0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1]])


    def predict(self, T, sigma_v, sigma_omega):

	self.predicted_x[2, 0] = self.estimated_x[2, 0] + self.estimated_x[4, 0] * T
	# norm angle [-pi pi]------------------------------------------------
	self.predicted_x[2, 0] = ((self.predicted_x[2, 0] + np.pi) % (2*np.pi)) - np.pi
	# -------------------------------------------------------------------
	self.predicted_x[0, 0] = self.estimated_x[0, 0] + self.estimated_x[3, 0] * T * np.cos(self.predicted_x[2, 0])
	self.predicted_x[1, 0] = self.estimated_x[1, 0] + self.estimated_x[3, 0] * T * np.sin(self.predicted_x[2, 0])
	self.predicted_x[3, 0] = self.estimated_x[3, 0]
	self.predicted_x[4, 0] = self.estimated_x[4, 0]

	

        # d_f is the jacobian of f for u = [v, omega]'
	#self.previous_x = self.predicted_x

        ang = T*self.estimated_x[4, 0] + self.estimated_x[2, 0]
        ang = ((ang + np.pi) % (2*np.pi))-np.pi

        d_f = np.matrix([[T*np.cos(ang), (-(T**2)*self.estimated_x[3, 0]*np.sin(ang))],
                        [T*np.sin(ang), (T**2)*self.estimated_x[3, 0]*np.cos(ang)],
                        [0, T],
                        [1, 0],
                        [0, 1]])

        # d_f_prime is the jacobian of f for x
        d_f_prime = np.matrix([[1, 0, -T*self.estimated_x[3, 0]*np.sin(ang), T*np.cos(ang), -(T**2)*self.estimated_x[3, 0]*np.sin(ang)],
                              [0, 1, T*self.estimated_x[3, 0]*np.cos(ang), T*np.sin(ang), (T**2)*self.estimated_x[3, 0]*np.cos(ang)],
                              [0, 0, 1, 0, T],
                              [0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 1]])

        var_Q = np.matrix([[sigma_v**2, 0], [0, sigma_omega**2]])
        Q = d_f * var_Q * d_f.T
        self.predicted_P = d_f_prime * self.estimated_P * d_f_prime.T + Q

	# previous update
        self.previous_x = self.predicted_x
        self.previous_P = self.predicted_P

    def Kalman_gain(self, odom_covariance, imu_covariance, vo_covariance, I_see_something):
	# if we see
	if I_see_something:
		C = self.C_full
		self.R = np.matrix(block_diag(odom_covariance, imu_covariance, vo_covariance))
	else:
		C = self.C_redu
		self.R = np.matrix(block_diag(odom_covariance, imu_covariance))
		
        # covariances here are extracted from /odom, /vo and /Imu
        # measurement in forme z = Cx + v , v is noise
        self.S = C * self.previous_P * C.T + self.R
        self.K = self.previous_P * C.T * np.linalg.inv(self.S)


    def estimate(self, measure, I_see_something):
        # calculates the error between measurement and prediction
        # update x and P
        # returns error
        # changed here vo to odom

	if I_see_something:
        	z = np.matrix([[measure.odom_x], [measure.odom_y],
                	      [measure.odom_theta], [measure.odom_v],
			      [measure.imu_theta], [measure.imu_omega],
                      	      [measure.vo_x], [measure.vo_y], [measure.vo_theta]])
		C = self.C_full
	else:
		z = np.matrix([[measure.odom_x], [measure.odom_y],
                      	      [measure.odom_theta], [measure.odom_v],
                              [measure.imu_theta], [measure.imu_omega]])
		C = self.C_redu

	# error calcualtion (becarfull here np matrixs)
	error = z - (C * self.previous_x)

        def nrmlz_angle(error):
            y = error
            y[2, 0] = ((y[2, 0] + np.pi) % (2*np.pi)) - np.pi
            return y

        error = nrmlz_angle(error)

	# we don't estimate first time
	if self.not_first_time:
		self.estimated_x = self.previous_x + self.K * error
		#print("this is ", self.estimated_x)
		# self.P = self.P - np.dot(K, np.dot(self.S, K.T))
		mat = np.eye(5, dtype=int) - self.K * C
		self.estimated_P = mat * self.previous_P
	else:
		self.not_first_time = True
		
        # self.P = np.dot(mat, np.dot(self.predicted_P, mat.T)) + np.dot(self.K, np.dot(self.R, self.K.T))
        # self.P = self.predicted_P - np.dot( self.K, np.dot(self.C, self.predicted_P))
        # print("i made estimation")

        # print(z)
	self.update_velocity()

        return error

    def callback_velocity(self, msg):
        # calculates velocity and updates x
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        v = np.sqrt(vx**2 + vy**2)
        self.estimated_x[3, 0] = v
        self.estimated_x[4, 0] = omega

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
        msg_odom.header.frame_id = '/base_footprint'
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
	self.I_see_something = True
	
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
	a = 0.1 # pose_covariance[35]
	b = 0.1 #(twist_covariance[0] + twist_covariance[7])
        self.odom_covariance = np.diag([pose_covariance[0],
                                        pose_covariance[7],
                                        a,
                                        b])

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
