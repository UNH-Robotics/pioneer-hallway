#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from actionlib import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

my_goal_status = 0
recovery_state = False
angular_z = 0 ;
override_z = 0.4 ;
cmd_vel_pub = rospy.Publisher("recovery_cmd_vel", Twist, queue_size = 10)

def turn_robot_around():
	global my_goal_status
	global angular_z
	global recovery_state
	global override_z
	if (my_goal_status == 1):
	#This state means the robot needs to be turned around.           
	#Reset global variables
		my_goal_status = 0
	#Turn the robot around (note coordinate transform reverses z-axis in gazebo)
		if (angular_z < 0.0):
			override_z = -0.4
		else:
			override_z = 0.4
		cmd_vel_data = Twist(Vector3(0,0,0), Vector3(0,0,override_z))
		cmd_vel_pub.publish(cmd_vel_data)
		recovery_state = True ;
		rospy.loginfo("Overriding /RosAria/cmd_vel, sending '[0,0,0] [0,0,%.1f]'",-cmd_vel_data.angular.z)
	elif (my_goal_status == 3):
	#This state means the robot has reached its goal.           
	#Reset global variables
		my_goal_status = 0
		cmd_vel_data = Twist(Vector3(0,0,0), Vector3(0,0,0))
		cmd_vel_pub.publish(cmd_vel_data)
		
def move_base_callback(data):
	global my_goal_status

	if data.status_list: 
		my_goal_status = data.status_list[0].status
	turn_robot_around()

def cmd_vel_callback(data):
	vel_data = data 
	global recovery_state
	if (recovery_state and vel_data.linear.x < 0.1):
		cmd_vel_data = Twist(Vector3(0,0,0), Vector3(0,0,override_z))
		cmd_vel_pub.publish(cmd_vel_data)
	elif (vel_data.linear.x == 0 and vel_data.linear.y == 0 and vel_data.linear.z == 0 and 
    	vel_data.angular.x == 0 and vel_data.angular.y == 0 and abs(vel_data.angular.z) <= 0.4):
		global angular_z
		angular_z = vel_data.angular.z
		turn_robot_around()
	else:
		cmd_vel_pub.publish(vel_data)
		recovery_state = False

def listener_node():
	rospy.init_node('listener_node', anonymous=True)

	rospy.Subscriber("move_base/status", GoalStatusArray, move_base_callback)
	rospy.Subscriber("controller_cmd_vel", Twist, cmd_vel_callback)	

	rospy.spin() 

if __name__ == '__main__':
	listener_node()
