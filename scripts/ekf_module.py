#! /usr/bin/env python

# title           : ekf_module.py
# description     : This module reads /vo, /odom, /imu and uses the extended kalman
#                   filtering for the fusion.
# author          : Salah Eddine Ghamri
# date            : 17-03-2018
# version         : 0.1
# usage           : must be run in navigation.launch
# notes           :
# python_version  : 2.6.7
# ==================================================================================

import rospy
import kalman_class

# initialisation------------------------------------

# x is of the form [x, y, theta, v, omega]'
x = [[0.0], [0.0], [0.0], [0.0], [0.0]]
# P is covariance size(x) x size(x)
P = [0.0, 0.0, 0.0, 0.0, 0.0]
# T time step, must be determined "ROS frequency"
t = 0.01  # not used
# variance of process noise
sigma_v = 0.1
sigma_omega = 0.1
# variance of measurement nooise
"""
sigma_odom_x = 2
sigma_odom_y = 2
sigma_odom_theta = 2
sigma_odom_v = 2
sigma_imu_theta = 2
sigma_imu_omega = 2
sigma_vo_x = 2
sigma_vo_y = 2
sigma_vo_theta = 2

odom_covariance = np.diag([sigma_odom_x**2, sigma_odom_y**2, sigma_odom_theta**2, sigma_odom_v**2])
imu_covariance = np.diag([sigma_imu_theta**2, sigma_imu_omega**2])
vo_covariance = np.diag([sigma_vo_x**2, sigma_vo_y**2, sigma_vo_theta**2])
"""
# --------------------------------------------------
if __name__ == "__main__":
    kalman = kalman_class.kalman_class(x, P)
    caller = kalman_class.caller()
    try:
        old_time = rospy.Time(0).now().to_sec()
        # loop while not shutdown
        while not rospy.is_shutdown():

            # time step for prediction
            new_time = rospy.Time(0).now().to_sec()
            T = new_time - old_time

            # predict the actual robot position
            kalman.predict(T, sigma_v, sigma_omega)

            # read time (for ros message)
            time = kalman.time_update()

            # read sensors
            caller.read_sensors()

            # Kalman gain calculation
            kalman.Kalman_gain(caller.odom_covariance, caller.imu_covariance, caller.vo_covariance)

            # Estimation, estimates and return estimation error.
            error = kalman.estimate(caller)

            # print(error[0],error[1],error[2])
            print("estimated_x is %r and the odom is %r" % (kalman.estimated_x[0, 0], caller.odom_x)),
            print("estimated_y is %r and the odom is %r" % (kalman.estimated_x[1, 0], caller.odom_x)),
            print("estimated_theta %r and the odom is %r" % (kalman.estimated_x[2, 0], caller.odom_x))

            # Publish on /odom_combined
            kalman.publish_message()
            old_time = new_time

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
