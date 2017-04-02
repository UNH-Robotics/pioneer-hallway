#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

twistPub = None
actionPub = None
enableButton = None
turboButton = None
cancelButton = None


def callback(sensor_data):
	if ((sensor_data.buttons[enableButton] == 0) & 
		(sensor_data.buttons[turboButton] == 0) &
		(sensor_data.buttons[2] == 0)):
		if (sensor_data.buttons[cancelButton] != 0):
			command = GoalID()
			actionPub.publish(command)
			rospy.loginfo("Waypoint cancelled")
			command = Twist()
			twistPub.publish(command)
		#rospy.loginfo("Teleop disabled")			
	elif (sensor_data.buttons[enableButton] != 0):
		rospy.loginfo("Teleop enabled")
	elif (sensor_data.buttons[turboButton] != 0):
		rospy.loginfo("Teleop turbo enabled")
	elif (sensor_data.buttons[2] != 0):
		command = Twist()
		command.linear.x = 15
		twistPub.publish(command)
		rospy.loginfo("other")

def teleop():
	global twistPub
	global actionPub
	global enableButton
	global turboButton
	global cancelButton
	enableButton = rospy.get_param('enableButton', 0)
	turboButton = rospy.get_param('turboButton', 5)
	cancelButton = rospy.get_param('cancelButton', 1)
	twistPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	actionPub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
	rospy.init_node('pioneer_teleop', anonymous=True)
	rospy.Subscriber('joy', Joy, callback)
	rospy.spin()
		

if __name__ == '__main__':
	teleop()
