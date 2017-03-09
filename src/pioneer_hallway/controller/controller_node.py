#!/usr/bin/env python
"""
Author: Tianyi Gu
Date: Mar / 7 / 2017
Desc: Controller node 
"""
import sys
sys.path.insert(0, "../../../doc/motionPrimitive/")
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from threading import Thread
from primutils import Primitive, read_primitives, read_primitives_with_duration
from collections import defaultdict
import itertools
import tf
from nav_msgs.msg import Odometry

actionQueue = []
motions = defaultdict(list)
duration = 0.0
currentTwist = Twist()


def executive_callback(data):
    global actionQueue
    rospy.loginfo(rospy.get_caller_id() + 'Executive give me the motion: %s',
                  data.data)
    actionQueue.append(data.data)

def executive_listener():
    rospy.Subscriber('controller_msg', String, executive_callback)
    rospy.spin()

def pose_callback(data):
    global currentTwist
    rospy.loginfo(rospy.get_caller_id() + 'Get latest pose info: \n' + 
                  "linear x: %.2f" % data.twist.linear.x + "\n" + 
                  "linear y: %.2f" % data.twist.linear.y+"\n"+
                  "linear z: %.2f" % data.twist.linear.z+"\n"+
                  "angular x: %.2f" % data.twist.angular.x+"\n"+
                  "angular y: %.2f" % data.twist.angular.y+"\n"+
                  "angular z: %.2f" % data.twist.angular.z+"\n")
    currentTwist = data.twist

def pose_listener():
    rospy.Subscriber('pose', Odometry, pose_callback)
    rospy.spin()

def move():
    global actionQueue
    #here,we publish actions to the topic 'cmd_vel'
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    speedListener =  tf.TransformListener()
    #rospy.init_node('controller_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        while actionQueue:
            motionStr = actionQueue.pop(0)
            currentMotion = motions[motionStr]
            motion = currentTwist
            motion.linear.x += currentMotion[0] * duration
            motion.angular.z += currentMotion[1] * duration
            rospy.loginfo("controller publish action: \n" + 
                          "linear x: %.2f" % (motion.linear.x) + "\n" + 
                          "linear y: %.2f" % motion.linear.y+"\n"+
                          "linear z: %.2f" % motion.linear.z+"\n"+
                          "angular x: %.2f" % motion.angular.x+"\n"+
                          "angular y: %.2f" % motion.angular.y+"\n"+
                          "angular z: %.2f" % motion.angular.z+"\n" +
                          "duration: " + str(duration))
            pub.publish(motion)
            rospy.sleep(rospy.Duration(duration))
        motion = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        rospy.logerr("Action Queue is empty!")
        pub.publish(motion)
        rate.sleep()

def init_motions():
    global duration
    (primitives, duration) = read_primitives_with_duration("../../../doc/motionPrimitive/primitives.txt")
    dupMotions = [[p.name, p.va, p.wa] for p in primitives]
    dupMotions.sort()
    filterMotions = [m for m, _ in itertools.groupby(dupMotions)]
    for i in filterMotions:
        motions[i[0]].append(i[1])
        motions[i[0]].append(i[2])
    #    print i
    #print motions["a6"]    

if __name__ == '__main__':
    init_motions();
    rospy.init_node('controller_node', anonymous=True)
    execListernerThread = Thread(target=executive_listener, args=())
    poseListernerThread = Thread(target=pose_listener, args=())
    #execListernerThread.setDaemon(True)
    execListernerThread.start()
    controllerPublisherThread = Thread(target=move, args=())
    #controllerPublisherThread.setDaemon(True)
    controllerPublisherThread.start()
    
    # try:
    #     move()
    # except rospy.ROSInterruptException:
    #     pass
