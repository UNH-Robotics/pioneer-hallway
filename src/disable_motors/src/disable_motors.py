#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

cmdVelPub = None

def serviceCallback(response):
	rospy.loginfo("Disabling motors")
	cmdVelPub.publish(Twist())
	return {}

def disable_motors():
	global cmdVelPub
	rospy.init_node('disable_motors')
	service = rospy.Service('disable_motors', Empty, serviceCallback)
	cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	rospy.loginfo("Service ready...")
	rospy.spin()
		
if __name__ == '__main__':
	disable_motors()
