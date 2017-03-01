#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from collections import namedtuple

Obstacle = namedtuple("Obstacle", "x y")
ObstacleDb = []

def obstacle_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "obstacle_msg: %s", data.data)

def obstacle_listener():
    rospy.init_node('exec_obs_listen', anonymous=True)
    rospy.Subscriber('obst_tracker', String, obstacle_callback)
    print "exec_obs_listen set up and spinning..."
    rospy.spin()

if __name__ == '__main__':
    obstacle_listener()
