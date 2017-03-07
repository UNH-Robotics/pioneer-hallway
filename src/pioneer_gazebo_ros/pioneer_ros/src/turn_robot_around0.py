#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from actionlib import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

my_goal_status = 0
cmd_vel_state = False

def turn_robot_around():
	global my_goal_status
	global cmd_vel_state
	if (my_goal_status == 1 and cmd_vel_state == True):
	#This state means the robot needs to be turned around.           
	#Reset global variables
		my_goal_status = 0
		cmd_vel_state = False
	#Turn the robot around
		cmd_vel_pub = rospy.Publisher("/pioneer/cmd_vel", Twist)
		cmd_vel_data = Twist(Vector3(1,0,0), Vector3(0,0,2))
		cmd_vel_pub.publish(cmd_vel_data)

def move_base_callback(data):
	global my_goal_status

	my_goal_status = data.status_list[0].status
	turn_robot_around()

def cmd_vel_callback(data):
	vel_data = data 
	if (vel_data.linear.x <= 0.2 and vel_data.linear.x >= -0.2 and vel_data.linear.y == 0 and vel_data.linear.z == 0 and 
    	vel_data.angular.x == 0 and vel_data.angular.y == 0 and vel_data.angular.z == 0):
		global cmd_vel_state	
		cmd_vel_state = True
		turn_robot_around()

def listener_node():
	rospy.init_node('listener_node', anonymous=True)

	rospy.Subscriber("move_base/status", GoalStatusArray, move_base_callback)
	rospy.Subscriber("/pioneer/cmd_vel", Twist, cmd_vel_callback)	

	rospy.spin() 

if __name__ == '__main__':
	listener_node()
