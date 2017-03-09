#!/usr/bin/env python
import rospy
from std_msgs.msg import String
    
def exec_control_pub():
    pub = rospy.Publisher('controller_msg', String, queue_size=10)
    rospy.init_node('exec_control_pub', anonymous=True)
    print "executive set up and spinning..."
    print "publishing controller_msg...forward..."
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        basic_command_msg = "forward"
        rospy.loginfo(basic_command_msg)
        pub.publish(basic_command_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        exec_control_pub()
    except rospy.ROSInterruptException:
        pass
