#!/usr/bin/env python
"""
Author: Tianyi Gu
Create Date: May / 9 / 2017
Desc: Controller move test 
"""

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from threading import Thread
from collections import defaultdict
import itertools
import tf
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
from datetime import datetime
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

def amclpose_callback(data):
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    h = euler[2]
    rospy.loginfo(rospy.get_caller_id() + 'Get latest pose info: \n' + 
                  "position x: %.7f" %data.pose.pose.position.x + "\n" +
                  "position y: %.7f" %data.pose.pose.position.y + "\n" +
                  # "orientation x: %.7f" %data.pose.pose.orientation.x + "\n" +
                  # "orientation y: %.7f" %data.pose.pose.orientation.y + "\n" +
                  # "orientation z: %.7f" %data.pose.pose.orientation.z + "\n" +
                  # "orientation w: %.7f" %data.pose.pose.orientation.w + "\n" +
                  "h: %.2f" %h)

def rosaria_twist_callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'Get latest twist info: \n' + 
                  "linear x: %.2f" % data.twist.twist.linear.x + "\n" + 
                  "angular z: %.2f" % data.twist.twist.angular.z)

def laser_callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'Get latest laster info: \n' + 
                  "middle: %.2f" % data.ranges[540] + "\n")
    

def move_test():
    #here,we publish actions to the topic 'cmd_vel_request'
    pub = rospy.Publisher('cmd_vel_request', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.3
    twist.angular.z = 0
    
    pub.publish(twist)
    rate = rospy.Rate(60)
    beginClock = time.time()    
    rospy.loginfo(beginClock)
    while((time.time() - beginClock) <= 1.0):
        pub.publish(twist)
        rate.sleep()
    rospy.loginfo(time.time())
    rospy.logerr("1 second!")

if __name__ == '__main__':
    rospy.init_node('move_test_node', anonymous=True)
    rospy.Subscriber('base_scan', LaserScan, laser_callback)
    rospy.Subscriber('RosAria/pose', Odometry, rosaria_twist_callback)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amclpose_callback)
    rospy.sleep(rospy.Duration(1.0))
    move_test()
