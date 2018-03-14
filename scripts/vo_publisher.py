#! /usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers

# a handler for a publisher
pub = rospy.Publisher('/vo',Odometry, queue_size=20)

#frame definition
frame_id = '/base_footprint'

#transformation de tf

transform = tf.TransformListener()

#covariance matrix, this must be determined ????
P = np.mat(np.diag([0.1]*6))
P = np.array(P).reshape(6, 6)
P_twist = np.mat(np.diag([1000000]*6))
P_twist = np.array(P_twist).reshape(6, 6)

alvar_matrices={}

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
                alvar_matrices[str(tag.id)] = matrix

if __name__=="__main__":
		rospy.init_node('vo_publisher')
                listener = tf.TransformListener()
                rate = rospy.Rate(100)
                print "O.K."
		while not rospy.is_shutdown():
			try:
				now = rospy.Time(0)
				now_stamp = rospy.Time.now()
				rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback_alvar_message)
				if '1' in alvar_matrices:
					tran_Rmark1 = alvar_matrices['1']
				else:
					continue
				# This is a well unknown tag in the map
				listener.waitForTransform('/map', '/marker_1', now, rospy.Duration(1))
				(tran, rott) = listener.lookupTransform('/map','/marker_1', now)
				tran_markmap = listener.fromTranslationRotation(tran,rott)
				rospy.loginfo(tran_markmap)

				# Here we calculate the robot postion based on the alvar and rviz
				robot_pos = np.dot(tran_markmap, tran_Rmark1)
				# extracting postion from the homogeneous 4x4 matrix 

				x = robot_pos[0,3]
				y = robot_pos[1,3]
				z = robot_pos[2,3]
				rospy.loginfo(x, y, z)

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


