#!/usr/bin/env python

import rospy
import math
import random
import time
from math import radians

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

# Reference - ros.wiki.org

def callback(data):

    if data.ranges[0] > 1:
        cmd.linear.x = 2.0
        cmd.angular.z = 0.0
    else:
        cmd.linear.x = 0.0
        cmd.angular.z = -1*radians(90)
    pub.publish(cmd)

     

rospy.init_node('evader', anonymous=True)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
sub = rospy.Subscriber("base_scan", LaserScan, callback)
cmd = Twist()
    
rospy.spin()

