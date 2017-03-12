#!/usr/bin/env python
"""
Author: Tianyi Gu
Create Date: Mar / 7 / 2017
Desc: Controller node 
---------------------- 
Update by: Tianyi Gu
update Date: Mar / 11 / 2017
Desc: implement random control
"""
import sys
sys.path.insert(0, "../../../doc/motionPrimitive/")
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_srvs.srv import Empty
from threading import Thread
from primutils import Primitive, read_primitives, read_primitives_with_duration
from collections import defaultdict
import itertools
import tf
from nav_msgs.msg import Odometry
import numpy as np
import math
import time

receivedAction = 'NotReceived'
receivedGoalState = None
motions = defaultdict(list)
duration = 0.0
currentState = None
changePlan = 0
positionWeight = 0.9
samplingNum = 200
sampleScale = 2.0

class State(object):
    def __init__(self, x, y, v, w, h, t):
        self.x = float(x)
        self.y = float(y)
        self.v = float(v)
        self.w = float(w)
        self.h = float(h)
        self.t = t

def executive_callback(data):
    global receivedAction
    global receivedGoalState
    global changePlan
    rospy.loginfo(rospy.get_caller_id() + 'Executive give me the motion and state:\n%s',data.data)
    strList = data.data.split(',')
    receivedAction = strList[0]
    receivedGoalState = State(strList[1], strList[2],strList[3],
                              strList[4], strList[5],strList[6])
    #changePlan = strList[7]

def executive_listener():
    rospy.Subscriber('controller_msg', String, executive_callback)
    rospy.spin()

def pose_callback(data):
    global currentState
    rospy.loginfo(rospy.get_caller_id() + 'Get latest pose info: \n' + 
                  "linear x: %.2f" % data.twist.linear.x + "\n" + 
                  "linear y: %.2f" % data.twist.linear.y+"\n"+
                  "linear z: %.2f" % data.twist.linear.z+"\n"+
                  "angular x: %.2f" % data.twist.angular.x+"\n"+
                  "angular y: %.2f" % data.twist.angular.y+"\n"+
                  "angular z: %.2f" % data.twist.angular.z+"\n" +
                  "position x: %.2f" %data.pose.pose.position.x + "\n" +
                  "position y: %.2f" %data.pose.pose.position.y + "\n" +
                  "orentation x: %.2f" %data.pose.pose.orientation.x + "\n" +
                  "orentation y: %.2f" %data.pose.pose.orientation.y + "\n" +
                  "orentation z: %.2f" %data.pose.pose.orientation.z + "\n" +
                  "orentation w: %.2f" %data.pose.pose.orientation.w + "\n")
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    h = euler[2]
    currenntState = State(data.pose.pose.position.x,
                          data.pose.pose.position.y,
                          data.twist.linear.x,
                          data.twist.angular.z,
                          h,
                          time.time())

def pose_listener():
    rospy.Subscriber('pose', Odometry, pose_callback)
    rospy.spin()

def sampling_based_controller(refAction, start, end, beginClock):
    deltaT = time.time() - beginClock
    goalOffset = float('inf')
    twist = Twist()
    vCandidate = np.random.normal(end.v, sampleScale, samplingNum)
    wCandidate = np.random.normal(end.w, sampleScale, samplingNum)
    for i in range(samplingNum):
        goalX = start.x + vCandidate[i] * deltaT * math.cos(
            start.w + (wCandidate[i] * deltaT) / 2)
        goalY = start.y + vCandidate[i] * deltaT * math.sin(
            start.w + (wCandidate[i] * deltaT) / 2)
        goalH = start.h + wCandidate[i] * deltaT
        disP = math.sqrt(math.pow(goalX - end.x, 2.0) +
                        math.pow(goalY - end.y, 2.0))
        disH = abs(goalH - end.h)
        totalDis = positionWeight * disP + (1 - positionWeight) * disH
        if totalDis < goalOffset:
            goalOffset = totalDis
            twist.linear.x = vCandidate[i]
            twist.angular.z = wCandidate[i]
    print "get new action: ", twist.linear.x, twist.angular.z
    return twist

def move():
    global receivedAction
    global receivedGoalState
    global changePlan
    global currentState
    currentState = State(1, 1, 1, 1, 1, 1)
    #here,we publish actions to the topic 'cmd_vel'
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # The local controller run at 60hz
    rate = rospy.Rate(60)
    while receivedAction != 'NotReceived' and receivedGoalState != None:
        beginClock = time.time()
        currentAction = receivedAction
        goalState = receivedGoalState
        receivedAction = 'NotReceived'
        receivedGoalState = None
        while((time.time() - beginClock) <= duration):
            if changePlan:
                changePlan = 0
                break
            motion= sampling_based_controller(motions[currentAction],
                                              currentState,
                                              goalState,
                                              beginClock)
            rospy.loginfo("controller publish action: \n" + 
                          "linear x: %.2f" % (motion.linear.x) + "\n" + 
                          "linear y: %.2f" % motion.linear.y+"\n"+
                          "linear z: %.2f" % motion.linear.z+"\n"+
                          "angular x: %.2f" % motion.angular.x+"\n"+
                          "angular y: %.2f" % motion.angular.y+"\n"+
                          "angular z: %.2f" % motion.angular.z+"\n" +
                          "duration: " + str(duration))
            pub.publish(motion)
            rate.sleep()
    rospy.logerr("Not Received Action!")
    disable_motors = rospy.ServiceProxy('disable_motors', Empty)
    #disable_motors.call()

def init_motions():
    global duration
    (primitives, duration) = read_primitives_with_duration(
        "../../../doc/motionPrimitive/primitives.txt")
    dupMotions = [[p.name, p.va, p.wa] for p in primitives]
    dupMotions.sort()
    filterMotions = [m for m, _ in itertools.groupby(dupMotions)]
    for i in filterMotions:
        motions[i[0]].append(i[1])
        motions[i[0]].append(i[2])

def wait_for_first_action():
    #sleep until received the first action from the executive
    global receivedAction
    while receivedAction == 'NotReceived':
        rospy.loginfo("wait for the first action from the executive")
        rospy.sleep(rospy.Duration(1))  

if __name__ == '__main__':
    init_motions();
    rospy.init_node('controller_node', anonymous=True)
    #rospy.wait_for_service('disable_motors')
    execListernerThread = Thread(target=executive_listener, args=())
    poseListernerThread = Thread(target=pose_listener, args=())
    controllerPublisherThread = Thread(target=move, args=())
    
    poseListernerThread.start()
    execListernerThread.start()
    wait_for_first_action()   
    controllerPublisherThread.start()

