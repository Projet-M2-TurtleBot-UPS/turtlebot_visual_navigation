#! /usr/bin/env python

#title           :vo_publisher.py
#description     :This will create the visual odometry based on Ar-alvar 
#		  readings, after conversion to odometry typed messages 
#		  they will be published on a topic named /vo.
#author          :Salah Eddine Ghamri
#date            :15-03-2018
#version         :0.2
#usage           :python pyscript.py
#notes           :The position of landmarks are stored here (change if needed).
#python_version  :2.6.7  
#==============================================================================
import rospy
import tf
import geometry_msgs.msg
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers

# msg format is Odometry 

msg = Odometry()

# a handler for a publisher
pub = rospy.Publisher('/vo',Odometry, queue_size=20)

#frame definition
frame_id = '/base_footprint'

#transformation de tf

transform = tf.TransformListener()

#covariance matrix, this must be determined ????
P = np.mat(np.diag([0.01]*6))
P = np.array(P).reshape(6, 6)
P_twist = np.mat(np.diag([1000000]*6))
P_twist = np.array(P_twist).reshape(6, 6)

alvar_matrices={}

# dictionary for landmarks postion on the map
# landmark coordinates are stored as 4x4 matrices
# in ldmark dictionary ldmark['marker_...'][2] 
ldmark = {'marker_0':((3.826, -0.720, 0.31),(0.0, 0.0, 1.0, -0.009)),
          'marker_1':((2.590, -5.707, 0.31),(0.0, 0.0, 0.990, 0.144)),
          'marker_2':((-0.949, -0.891, 0.31), (0.0, 0.0, -0.792, 0.611)),
          'marker_3':((-1.808,-4.031,0.31), (0.0, 0.0,0.608,0.793)),
          'marker_4':((-5.503, 0.251, 0.31), (0.0, 0.0, -0.086, 0.996)),
          'marker_5':((-6.086, -1.588, 0.31),( 0.0, 0.0, -0.105, 0.994)),
          'marker_6':((-6.343, -2.841, 0.31),( 0.0, 0.0, -0.144, 0.989)),
          'marker_7':((-10.311, 0.256, 0.31),( 0.0, 0.0, -0.117, 0.993))}
for key in ldmark:
	ldmark[key] = (ldmark[key][0],ldmark[key][1],
		      transform.fromTranslationRotation(ldmark[key][0], ldmark[key][1]))

def callback_alvar_message(message):
        for tag in message.markers:
                Trans = (tag.pose.pose.position.x,
                        tag.pose.pose.position.y,
                        tag.pose.pose.position.z)
                Rot = (tag.pose.pose.orientation.x,
                        tag.pose.pose.orientation.y,
                        tag.pose.pose.orientation.z,
                        tag.pose.pose.orientation.w)
                matrix = transform.fromTranslationRotation(Trans, Rot)
                alvar_matrices["marker_"+str(tag.id)] = matrix

if __name__=="__main__":
		rospy.init_node('vo_publisher')
                listener = tf.TransformListener()
                rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			try:
				now = rospy.Time(0)
				now_stamp = rospy.Time.now()
				rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback_alvar_message)
				if len(alvar_matrices) >=1 :
					# here more regorous method need to be taken
					dtctd = min(alvar_matrices.keys())
					print dtctd
					tran_Rmark1 = alvar_matrices[dtctd]
				else:
					continue
				
				"""
				# This is a well unknown tag in the map
				listener.waitForTransform('/map', '/marker_1', now, rospy.Duration(1))
				(tran, rott) = listener.lookupTransform('/map','/marker_1', now)
				tran_markmap = listener.fromTranslationRotation(tran,rott)
				rospy.loginfo(tran_markmap)
				"""
				
				# Here we call the position of the detected landmark
				tran_markmap = ldmark[dtctd][2]
				# Here we calculate the robot postion based on the alvar and rviz
				robot_pos = np.dot(tran_markmap, tran_Rmark1)
				# extracting postion from the homogeneous 4x4 matrix 

				x = robot_pos[0,3]
				y = robot_pos[1,3]
				z = robot_pos[2,3]

				# extracting Quaternion from the homogeneous 4x4 matrix 
				quatern = tf.transformations.quaternion_from_matrix(robot_pos)

				# Constructing the message
				msg.header.stamp = now_stamp
				msg.header.frame_id = frame_id
				#msg.header.child_frame_id = child_frame_id
				msg.pose.pose.position = Point(x,y,z)
				msg.pose.pose.orientation = Quaternion(*quatern)
				msg.pose.covariance = tuple(P.ravel().tolist())
				msg.twist.covariance = tuple(P_twist.ravel().tolist())
				# publishing the message
				pub.publish(msg)
				
			except (tf.LookupException, tf.ConnectivityException,rospy.ROSInterruptException):
                        	 continue

                        rate.sleep()


