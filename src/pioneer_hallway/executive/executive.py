#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

world_map = None
obstacles = None
position_est = None

def obstacle_callback(data):
    rospy.loginfo(rospy.get_call_id() + "obstacle_msg: ", data.data)

def obstacle_listener():
    rospy.init_node('exec_obs_listen', anonymous=True)
    rospy.Subscriber('obst_tracker', String, obstacle_callback)
    print "exec_obs_listen set up and spinning..."
    
def controller_callback(data):
    rospy.loginfo(rospy.get_call_id() + "controller_msg: ", data.data)

def controller_listener():
    rospy.init_node('exec_control_listen', anonymous=True)
    rospy.Subscriber('controller', String, controller_callback)
    print "exec_control_listen set up and spinning..."
    
def executive():
    pub = rospy.Publisher('controller_msg', String, queue_size=10)
    rospy.init_node('executive', anonymous=True)
    print "executive set up and spinning..."
    print "publishing controller_msg...forward..."
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        basic_command_msg = "forward"
        rospy.loginfo(basic_command_msg)
        pub.publish(basic_command_msg)
        rate.sleep()

if __name__ == '__main__':
    obstacle_listener()
    controller_listener()
    try:
        executive()
    except rospy.ROSInterruptException:
        pass
