#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def test_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "msg: %s", data.data)

def obstacle_listener():
    rospy.init_node('exec_test_listen', anonymous=True)
    rospy.Subscriber('controller_msg', String, test_callback)
    print "exec_test_listen set up and spinning..."
    rospy.spin()

if __name__ == '__main__':
    obstacle_listener()
