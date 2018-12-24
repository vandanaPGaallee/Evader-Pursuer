#!/usr/bin/env python

import rospy
import math
import random
import time
from math import radians

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import tf

# Reference - ros.wiki.org

def tfbroadcaster(msg, turtlename):
    br = tf.TransformBroadcaster()
    msg_pos = msg.pose.pose.position
    msg_or = msg.pose.pose.orientation
    br.sendTransform((msg_pos.x,msg_pos.y,0),(msg_or.x, msg_or.y, msg_or.z, msg_or.w),rospy.Time.now(),turtlename,"world")

if __name__ == '__main__':
	rospy.init_node('pursuer_controller', anonymous=True)
	pub = rospy.Publisher('robot_1/cmd_vel', Twist, queue_size=5)
	rospy.Subscriber('/robot_1/odom', Odometry, tfbroadcaster, 'robot_1')
	listener = tf.TransformListener()
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		try:
			now = rospy.Time.now()
			rate.sleep()
			past = now - rospy.Duration(1.0)
			listener.waitForTransformFull("/robot_1", now, "/robot_0", past, "/world",rospy.Duration(1.0))
			(trans,rot) = listener.lookupTransformFull("/robot_1", now, "/robot_0", past, "/world")
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):continue
		angular = 4 * math.atan2(trans[1], trans[0]) # Angular speed 
		linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2) #distance calculations from origin using distance form roswiki
		cmd = Twist()
		cmd.linear.x = linear
		cmd.angular.z = angular
		pub.publish(cmd)

		rate.sleep()
	rospy.spin()