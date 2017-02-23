#!/usr/bin/env python
import rospy
from std_msgs.msg import String

obstacles = None

def obstacle_callback(data):
    rospy.loginfo(rospy.get_call_id() + "obstacle_msg: ", data.data)

def obstacle_listener():
    rospy.init_node('exec_obs_listen', anonymous=True)
    rospy.Subscriber('obst_tracker', String, obstacle_callback)
    print "exec_obs_listen set up and spinning..."
    rospy.spin()

if __name__ == '__main__':
    obstacle_listener()
