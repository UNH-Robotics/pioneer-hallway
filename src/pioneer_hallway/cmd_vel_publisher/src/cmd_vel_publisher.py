#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

#global variables
cmdVelPub = None
pubEnabled = True

def disableCmdVelPubServiceCallback(response):
	global pubEnabled
	pubEnabled = False
	rospy.loginfo("Publisher disabled")
	cmdVelPub.publish(Twist())
	return {}
	
def cmdVelCallback(twist):
	if pubEnabled == True:
		cmdVelPub.publish(twist)
	return {}

def cmd_vel_publisher():
	global cmdVelPub
	rospy.init_node('pioneer_hallway_cmd_vel_publisher', anonymous=False)
	cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	service = rospy.Service('disable_cmd_vel_publisher', Empty, disableCmdVelPubServiceCallback)
	rospy.Subscriber('cmd_vel_request', Twist, cmdVelCallback)
	rospy.loginfo('Publisher node ready.')
	rospy.spin()
		

if __name__ == '__main__':
	cmd_vel_publisher()
