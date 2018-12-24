#!/usr/bin/env python

import rospy
import math
import random
import time
from math import radians
import tf

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

# Reference - ros.wiki.org

def detectObstacles(data):
	
    if data.ranges[0] > 1:
        cmd.linear.x = 0.5
        cmd.angular.z = 0.0
    else:
        cmd.linear.x = 0.0
        cmd.angular.z = radians(90)

    pub.publish(cmd)

def tfbroadcaster(msg, turtlename):
    br = tf.TransformBroadcaster()
    msg_pos = msg.pose.pose.position
    msg_or = msg.pose.pose.orientation
    br.sendTransform((msg_pos.x,msg_pos.y,0),(msg_or.x, msg_or.y, msg_or.z, msg_or.w),rospy.Time.now(),turtlename,"world")

if __name__ == '__main__':
    rospy.init_node('evader_controller', anonymous=True)
    pub = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=5)
    sub = rospy.Subscriber("robot_0/base_scan", LaserScan, detectObstacles)
    rospy.Subscriber('/robot_0/odom', Odometry, tfbroadcaster, 'robot_0')
    cmd = Twist()

    rospy.spin()