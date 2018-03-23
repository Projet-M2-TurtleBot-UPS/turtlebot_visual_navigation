#! /usr/bin/env python

# title           : ekf_module.py
# description     : This module reads /vo, /odom, /imu and uses the extended kalman
#                   filtering for the fusion.
# author          : Salah Eddine Ghamri
# date            : 17-03-2018
# version         : 0.4
# usage           : must be run in navigation.launch
# notes           : Rate affects response
# python_version  : 2.6.7
# ==================================================================================

import rospy
import kalman_class

# initialisation------------------------------------

# node initialisation
rospy.init_node("ekf_module")
frequency = 100.00
Rate = rospy.Rate(frequency)  # very dangerous pay attention
t = 0
step = 1/frequency
# x is of the form [x, y, theta, v, omega]'
x = [[0.0], [0.0], [0.0], [0.0], [0.0]]
# P is covariance size(x) x size(x)
P = [0.0, 0.0, 0.0, 0.0, 0.0]
# variance of process noise
sigma_v = 10
sigma_omega = 10
# variance of measurement noise
# --------------------------------------------------

kalman = kalman_class.kalman_class(x, P)
caller = kalman_class.caller()

if __name__ == "__main__":
    try:
        old_time = rospy.Time().now().to_sec()
        # loop while not shutdown
        while not rospy.is_shutdown():

            # time step for prediction
            new_time = rospy.Time().now().to_sec()
            time = kalman.time_update()  # for message
            T = new_time - old_time
	    T = (1/frequency)*10
	    rospy.loginfo(T)

            # read sensors now
            caller.read_sensors()

            # Estimation, estimates and return estimation error.
            error = kalman.estimate(caller)
	    
	    # check tags visibility 
            # predict the next robot position
            kalman.predict(T, sigma_v, sigma_omega)

            # Publish on /odom_combined
            kalman.publish_message()
            old_time = new_time
	    #rospy.sleep(10)
	    Rate.sleep()

    except rospy.ROSInterruptException:
        pass
