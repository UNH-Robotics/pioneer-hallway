#!/usr/bin/env python
"""
Author: Tianyi Gu
Create Date: Mar / 7 / 2017
Desc: Controller node 
---------------------- 
Update by: Tianyi Gu
update Date: Mar / 11 / 2017
Desc: Implement model predictive control
---------------------- 
Update by: Tianyi Gu
update Date: Mar / 27 / 2017
Desc: 1.  update interface to cmd_vel_publisher node 
          and disable_cmd_vel_publish service ( for emegency stop)
      2.  update state from x, y, v, w, h, t to remove w
      3.  update model predictive control to include motion primitive info
"""
import sys
sys.path.insert(0, "../../../doc/motionPrimitive/")
#sys.path.insert(0, "doc/motionPrimitive/")
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
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
from datetime import datetime
from nav_msgs.msg import Path

receivedAction = 'NotReceived'
receivedGoalState = None
receivedAction2= 'NotReceived'
receivedGoalState2 =None
motions = defaultdict(list)
duration = 0.0
currentState = None
changePlan = 0
getNewState = 0
#positionWeight = 0.33
#headingWeight = 0.33
#velocityWeight = 0.33
samplingNum = 3000
sampleScale_pos = 0.1
sampleScale_h = 0.01
disUnit_postion =0.01
disUnit_heading = 3 * math.pi / 180
disUnit_velocity = 0.15
pathFileName = None
trajectoryFileName = None
controllerLogName = None
plannerPath = None
plannerPathPub = None

class State(object):
    def __init__(self, x, y, v, h, t, w = 0):
        self.x = float(x)
        self.y = float(y)
        self.v = float(v)
        self.h = float(h)
        self.t = t
        self.w = float(w)
        
    def set_pose(self, x, y, h, t):
        self.x = float(x)
        self.y = float(y)
        self.h = float(h)
        self.t = t

    def set_twist(self, v, w, t):
        self.v = float(v)
        self.w = float(w)
        self.t = t

def executive_callback(data):
    global receivedAction
    global receivedGoalState
    global changePlan
    global plannerPath
    global plannerPathPub
    rospy.loginfo(rospy.get_caller_id() + 'Executive give me the motion and state:\n%s',data.data)
    twoStr=data.data.split(';')
    strList = twoStr[0].split(',')
    receivedAction = strList[0]
    receivedGoalState = State(strList[1], strList[2],strList[3],
                              strList[4], strList[5])
    if len(twoStr)>1:
        strList = twoStr[1].split(',')
        receivedAction2 = strList[0]
        receivedGoalState2 = State(strList[1], strList[2],strList[3],
                                   strList[4], strList[5])
    else:
        receivedAction2='NotReceived'
        receivedGoalState2=None
    #changePlan = strList[7]
    # with open(pathFileName, 'a') as f:
    #     f.write(data.data)

def executive_listener():
    rospy.Subscriber('controller_msg', String, executive_callback)
    #rospy.spin()

def rosaria_twist_callback(data):
    global currentState
    # rospy.loginfo(rospy.get_caller_id() + 'Get latest twist info: \n' + 
    #               "linear x: %.2f" % data.twist.twist.linear.x + "\n" + 
    #               "angular z: %.2f" % data.twist.twist.angular.z)
    currentState.set_twist(data.twist.twist.linear.x,
                           data.twist.twist.angular.z, 
                           time.time())
        
def pose_callback(data):
    global currentState
    global getNewState
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    h = euler[2]
    # rospy.loginfo(rospy.get_caller_id() + 'Get latest pose info: \n' + 
    #               "linear x: %.2f" % data.twist.twist.linear.x + "\n" + 
    #               "angular z: %.2f" % data.twist.twist.angular.z+"\n" +
    #               "position x: %.2f" %data.pose.pose.position.x + "\n" +
    #               "position y: %.2f" %data.pose.pose.position.y + "\n" +
    #               "heading h: %.2f" %h )
    #update only received a different state
    if h!=currentState.h:
        currentState = State(data.pose.pose.position.x,
                             data.pose.pose.position.y,
                             data.twist.twist.linear.x,
                             h,
                             time.time(), 
                             data.twist.twist.angular.z)
        getNewState = 1

    
def amclpose_callback(data):
    global currentState
    global getNewState
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    h = float(euler[2])
    # rospy.loginfo(rospy.get_caller_id() + 'Get latest pose info: \n' + 
    #               "position x: %.7f" %data.pose.pose.position.x + "\n" +
    #               "position y: %.7f" %data.pose.pose.position.y + "\n" +
    #               # "orientation x: %.7f" %data.pose.pose.orientation.x + "\n" +
    #               # "orientation y: %.7f" %data.pose.pose.orientation.y + "\n" +
    #               # "orientation z: %.7f" %data.pose.pose.orientation.z + "\n" +
    #               # "orientation w: %.7f" %data.pose.pose.orientation.w + "\n" +
    #               "h: %.2f" %h)
    #update only received a different state
    if h!=currentState.h or \
       data.pose.pose.position.x != currentState.x or \
       data.pose.pose.position.y != currentState.y:
        currentState.set_pose(data.pose.pose.position.x,
                              data.pose.pose.position.y,
                              h,
                              time.time())
        getNewState = 1
    # with open(trajectoryFileName, 'a') as f:
    #     f.write( "%.2f" %data.pose.pose.position.x + "," +
    #              "%.2f" %data.pose.pose.position.y + "," + 
    #              "%.2f" %h + '\n')

def twist_callback(data):
    global currentState
    rospy.loginfo(rospy.get_caller_id() + 'Get latest twist info: \n' + 
                  "linear x: %.2f" % data.linear.x + "\n" + 
                  "linear y: %.2f" % data.linear.y+"\n"+
                  "linear z: %.2f" % data.linear.z+"\n"+
                  "angular x: %.2f" % data.angular.x+"\n"+
                  "angular y: %.2f" % data.angular.y+"\n"+
                  "angular z: %.2f" % data.angular.z+"\n" )
    currentState.set_twist(data.linear.x,
                           data.angular.z,
                           time.time())

    
def pose_listener():
    global currentState
    currentState = State(1, 1, 1, 1, 1, 1)
    rospy.Subscriber('RosAria/pose', Odometry, rosaria_twist_callback)
    #rospy.Subscriber('RosAria/pose', Odometry, pose_callback)
    #rospy.Subscriber('odom', Odometry, pose_callback)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amclpose_callback)
    #rospy.Subscriber('cmd_vel', Twist, twist_callback)
    #rospy.spin()

def sampling_based_controller(refAction, start, end, endClock):
    deltaT = endClock - time.time()
    goalOffset = float('inf')
    goalX_est = 0
    goalY_est = 0
    goalH_est = 0
    twist = Twist()
    #check if over shooting
    disSE = math.sqrt(math.pow(start.x - end.x, 2.0) +
                        math.pow(start.y - end.y, 2.0))
    if disSE < 0.01:
        twist.linear.x = 0
        twist.angular.z = 0
        print "over shooting"
    else: 
        vCandidate = np.random.uniform(0, 0.75, samplingNum)
        wCandidate = np.random.uniform(-1.73, 1.73, samplingNum)
        for i in range(samplingNum):
            goalX = start.x + vCandidate[i] * deltaT * math.cos(
                start.h)
            goalY = start.y + vCandidate[i] * deltaT * math.sin(
                start.h)
            #+ wCandidate[i] * deltaT / 2
            goalH = start.h + wCandidate[i] * deltaT
            #(1-vCandidate[i]/3)*
            disP = math.sqrt(math.pow(goalX - end.x, 2.0) +
                             math.pow(goalY - end.y, 2.0))
            #disH = abs(goalH - end.h) % (2 * math.pi)
            disH = abs(goalH - end.h)
            disV = abs(vCandidate[i] - end.v)
            totalDis = disP / disUnit_postion + \
                       disH / disUnit_heading + \
                       disV / disUnit_velocity
            if totalDis < goalOffset:
                goalOffset = totalDis
                twist.linear.x = vCandidate[i]
                twist.angular.z = wCandidate[i]
                goalX_est = goalX
                goalY_est = goalY
                goalH_est = goalH
    print "get new action to: ", end.x, end.y, end.v, end.h, "\naction: ", twist.linear.x, twist.angular.z, "\ngoalEST: ", goalX_est, goalY_est, goalH_est, "\noffset", goalOffset, "\nstartvw: ", start.v, start.w, "\nstartxyh: ", start.x, start.y, start.h, "\nprim: ", refAction[0], refAction[1], "\ndT: ", deltaT
    return twist



def model_predictive_controller(refAction, start, end, endClock):
    if end.h > math.pi:
        end.h = end.h - 2 * math.pi
    deltaT = endClock - time.time()
    goalOffset = float('inf')
    goalX_est = 0
    goalY_est = 0
    goalH_est = 0
    twist = Twist()
    refv = start.v + refAction[0] * deltaT
    vCandidate = np.random.normal(refv, sampleScale_pos, samplingNum)
    vCandidate = vCandidate.tolist()
    vCandidate.append(refv)
    #print vCandidate
    #print
    dh = (end.h - start.h) / duration
    if dh > 5.3:
        dh = math.pi
    elif dh <-5.3:
        dh = -5.3
    wCandidate = np.random.normal(dh, sampleScale_h, samplingNum)
    #wCandidate = np.random.uniform( -math.pi / 2, math.pi, samplingNum)
    #wCandidate = np.random.normal(start.w + refAction[1] * deltaT,
    #                              sampleScale, samplingNum)
    
    wCandidate = wCandidate.tolist()
    wCandidate.append(dh)
    #print wCandidate
    for i in range(samplingNum + 1):
        if vCandidate[i] < 0:
            vCandidate[i] = 0
        #print vCandidate[i], wCandidate[i]
        #acturalV = (start.v + vCandidate[i]) / 2
        #acturalW = (start.w + wCandidate[i]) / 2
        acturalV = vCandidate[i]
        acturalW = wCandidate[i]
        goalX = start.x + acturalV * deltaT * math.cos(
            start.h + (acturalW * deltaT) / 2)
        goalY = start.y + acturalV * deltaT * math.sin(
            start.h + (acturalW * deltaT) / 2)
        goalH = start.h + acturalW * deltaT
        disP = math.sqrt(math.pow(goalX - end.x, 2.0) +
                        math.pow(goalY - end.y, 2.0))
        #disH = abs(goalH - end.h) % (2 * math.pi)
        disH = abs(goalH - end.h)
        disV = abs(acturalV - end.v)
        totalDis = disP / disUnit_postion + \
                   disH / disUnit_heading + \
                   disV / disUnit_velocity
        if i == samplingNum:
            print "refresutl: ",acturalV, acturalW, goalX, goalY, goalH, totalDis
        if totalDis < goalOffset:
            goalOffset = totalDis
            twist.linear.x = vCandidate[i]
            twist.angular.z = wCandidate[i]
            goalX_est = goalX
            goalY_est = goalY
            goalH_est = goalH
    print "get new action to: ", end.x, end.y, end.v, end.h,"\nrefaction: ", start.v + refAction[0] * deltaT, dh, "\naction: ", twist.linear.x, twist.angular.z, "\ngoalEST: ", goalX_est, goalY_est, goalH_est, "\noffset", goalOffset, "\nstartvw: ", start.v, start.w, "\nstartxyh: ", start.x, start.y, start.h, "\nprim: ", refAction[0], refAction[1], "\ndeltaT","%.2f" % deltaT
    return twist

class Controller(object):
    def __init__(self, refAction, start, end, endClock):
        self.refAction = refAction
        self.start = start
        self.endClock = endClock
        self.deltaT = endClock - time.time()
        if end.h > math.pi:
            end.h = end.h - 2 * math.pi
        self.end = end

    def forwardSim(self, x, y, vwType):
        v = 0
        w = 0
        if vwType == "v":
            v = x
            w = y
        else:
            v = y
            w = x            
        goalX = self.start.x + \
                v * self.deltaT * math.cos(self.start.h + (w * self.deltaT) / 2)
        goalY = self.start.y + \
                v * self.deltaT * math.sin(self.start.h + (w * self.deltaT) / 2)
        goalH = self.start.h + w * self.deltaT
        disP = math.sqrt(math.pow(goalX - self.end.x, 2.0) +
                        math.pow(goalY - self.end.y, 2.0))
        disH = abs(goalH - self.end.h)
        disV = abs(v - self.end.v)
        totalDis = disP / disUnit_postion + \
                   disH / disUnit_heading + \
                   disV / disUnit_velocity
        return totalDis

    def bracketVW(self, x1, x2, y, vwType):
        m = (x1 + x2) / 2
        f1 = self.forwardSim(x1, y, vwType)
        fm = self.forwardSim(m, y, vwType)
        f2 = self.forwardSim(x2, y, vwType)
        if fm < f1 and fm < f2:
            return (x1, x2, m)
        elif f1 < fm and f1 < f2:
            return (2 * x1 - x2,m, x1)
        elif f2 < fm and f2 < f1:
            return (m, 2 * x2 - x1, x2)
        return (-1,-1,-1)

    def refineEstVW(self, l, m, r, y, vwType, minRange, minDeltaF):
        deltaF = self.forwardSim(m, y, vwType)
        i = 0
        bestF = 1000
        while abs(l - r) > minRange and deltaF > minDeltaF:
            #print "i f refine", i, bestF
            lm = (l + m) / 2
            fm = self.forwardSim(m, y, vwType)
            flm = self.forwardSim(lm, y, vwType)
            if flm < fm:
                r = m
                m = lm
                deltaF = abs(fm - flm)
                bestF = flm
                i += 1
                continue
            mr = (m + r) / 2
            fmr = self.forwardSim(mr, y, vwType)
            if fmr < fm:
                l = m
                m = mr
                deltaF = abs(fm - fmr)
                bestF = fmr
                i += 1
                continue
            break
        return (l, m, r, i, bestF)

    def bisection_search(self, v1, v2, w1, w2, vMax, wMax):
        i = 100
        vl = v1
        vm = v1
        vr = v2
        wl = w1
        wm = w1
        wr = w2
        bestF = 1000
        vwType = "v"
        if v1 == v2:
            vwType = "w"
            vl = -vMax
            vr = vMax
        elif w1 == w2:
            vwType = "v"
            wl = -wMax
            wr = wMax
        j = 0
        while(i > 0):
            #print "j, i: ", j, i, vwType
            tempF = 1000
            if vwType == "v":
                if j == 0: 
                    (vl, vm, vr) = self.bracketVW(vl, vr, wm, vwType)
                (vl, vm, vr, i, tempF) = self.refineEstVW(vl, vm, vr, \
                                                          wm, vwType, 0.01, 0.01)
                vwType = "w"
            else:
                if j == 0: 
                    (wl, wm, wr) = self.bracketVW(wl, wr, vm, vwType)
                (wl, wm, wr, i, tempF) = self.refineEstVW(wl, wm, wr, \
                                                          vm, vwType, 0.01, 0.01)
                vwType = "v"
            if i > 0:
                bestF = tempF
            j += 1
            #print bestF
            #print vm, wm, vl, vr, wl, wr
        return(vm, wm, bestF)

    def bisection_search_controller(self):
        goalOffset1 = float('inf')
        goalOffset2 = float('inf')
        goalX_est = 0
        goalY_est = 0
        goalH_est = 0
        twist = Twist()
        vCandidate = []
        wCandidate = []
        vMean = self.start.v + self.refAction[0] * self.deltaT
        vSmallD = 0.2
        vMax = 2.2
        dh = (self.end.h - self.start.h) / duration
        if dh > 5.3:
            dh = math.pi
        elif dh <-5.3:
            dh = -5.3
        wMean = dh
        wSmallD = 0.2
        wMax = 5.3 #300 degree per sec
        vCandidate.append(-vMax)
        vCandidate.append(vMean - vSmallD)
        vCandidate.append(vMean)
        vCandidate.append(vMean + vSmallD)
        vCandidate.append(vMax)
        wCandidate.append(-wMax)
        wCandidate.append(wMean - wSmallD)
        wCandidate.append(wMean)
        wCandidate.append(wMean + wSmallD)
        wCandidate.append(wMax)
        bestV = 0
        bestV2 = 0
        bestW = 0
        bestW2 = 0
        for i, v in enumerate(vCandidate):
            for j, w in enumerate(wCandidate):
                totalDis = self.forwardSim(v, w, "v")
                if totalDis < goalOffset1:
                    goalOffset2 = goalOffset1
                    bestV2 = bestV
                    bestW2 = bestW
                    goalOffset1 = totalDis
                    bestV = v
                    bestW = w
                elif totalDis < goalOffset2:
                    goalOffset2 = totalDis
                    bestV2 = v
                    bestW2 = w
        #print bestV, bestV2, bestW, bestW2
        (tv, tw, bestF) = self.bisection_search(bestV, bestV2, bestW, bestW2, \
                                                vMax, wMax)
        twist.linear.x = tv
        twist.angular.z = tw
        print "goal state: ", self.end.x, self.end.y, self.end.v, self.end.h, \
            "\naction: ", twist.linear.x, twist.angular.z, \
            "\noffset", bestF,\
            "\nstartvw: ", self.start.v, self.start.w, \
            "\nstartxyh: ", self.start.x, self.start.y, self.start.h, \
            "\nprim: ", self.refAction[0], self.refAction[1], \
            "\ndeltaT","%.2f" % self.deltaT
        return twist

def move():
    global receivedAction
    global receivedGoalState 
    global changePlan
    global currentState
    global getNewState
    #here,we publish actions to the topic 'cmd_vel_request'
    pub = rospy.Publisher('cmd_vel_request', Twist, queue_size=10)
    #pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # The local controller run at 60hz
    rate = rospy.Rate(200)
    i = 1
    motion = None
    while receivedAction != 'NotReceived' and receivedGoalState != None:
        print i
        i += 1
        beginClock = time.time()
        endClock = beginClock + duration
        currentAction = receivedAction
        goalState = receivedGoalState
        receivedAction = 'NotReceived'
        receivedGoalState = None
        pose = PoseStamped()
        pose.pose.position.x = goalState.x
        pose.pose.position.y = goalState.y
        plannerPath.poses.append(pose)
        plannerPath.header.stamp = rospy.Time.now()
        plannerPathPub.publish(plannerPath)
        print len(plannerPath.poses)
        pubrate = rospy.Rate(1000)
        getNewState=1#reset received current, to prevent dead lock
        while((time.time() - beginClock) <= duration):
            if changePlan:
                changePlan = 0
                break
            if getNewState: 
                # motion= sampling_based_controller(motions[currentAction],
                #                                   currentState,
                #                                   goalState,
                #                                   endClock)
                motion= model_predictive_controller(motions[currentAction],
                                                    currentState,
                                                    goalState,
                                                    endClock)
                # biController = Controller(motions[currentAction],
                #                                     currentState,
                #                                     goalState,
                #                                     endClock)
                # motion = biController.bisection_search_controller()
                # rospy.loginfo("controller publish action: \n" + 
                #               "linear x: %.2f" % (motion.linear.x) + "\n" + 
                #               "linear y: %.2f" % motion.linear.y+"\n"+
                #               "linear z: %.2f" % motion.linear.z+"\n"+
                #               "angular x: %.2f" % motion.angular.x+"\n"+
                #               "angular y: %.2f" % motion.angular.y+"\n"+
                #               "angular z: %.2f" % motion.angular.z+"\n" +
                #               "duration: " + str(duration))
                pub.publish(motion)
                getNewState = 0
                pubrate.sleep()
            else:
                pub.publish(motion)
                pubrate.sleep()
                #rate.sleep()
            #rate.sleep()
    rospy.logerr("Not Received Action!")
    disable_motors = rospy.ServiceProxy('disable_cmd_vel_publisher', Empty)
    disable_motors.call()
      
def init_motions():
    global duration
    #prim_file = rospy.get_param("primitive_file")
    prim_file = "../../../doc/motionPrimitive/primitives.txt"
    (primitives, duration) = read_primitives_with_duration(prim_file)
    dupMotions = [[p.name, p.va, p.wa] for p in primitives]
    dupMotions.sort()
    filterMotions = [m for m, _ in itertools.groupby(dupMotions)]
    for i in filterMotions:
        motions[i[0]].append(i[1])
        motions[i[0]].append(i[2])
        
def init_path_output():
    # global pathFileName
    # global trajectoryFileName
    # suffix = datetime.now().strftime("%m%d%Y_%H%M%S") 
    # pathFileName = "path_" + suffix + ".co"
    # trajectoryFileName = "trajectory_" + suffix + ".co"
    # controllerLogName = "ctrl_" + suffix + ".log"
    global plannerPathPub
    global plannerPath
    plannerPathPub = rospy.Publisher('planner_path_publisher', Path, queue_size=10)
    plannerPath = Path()
    plannerPath.header.frame_id = "map"
    

def wait_for_first_action():
    #sleep until received the first action from the executive
    global receivedAction
    while receivedAction == 'NotReceived':
        rospy.loginfo("waiting for the first action from the executive")
        rospy.sleep(rospy.Duration(0.01))  

if __name__ == '__main__':
    init_motions()
    init_path_output()
    rospy.init_node('controller_node', anonymous=True)
    rospy.wait_for_service('disable_cmd_vel_publisher')
    #execListernerThread = Thread(target=executive_listener, args=())
    #poseListernerThread = Thread(target=pose_listener, args=())
    #controllerPublisherThread = Thread(target=move, args=())
    executive_listener()
    pose_listener()

    #poseListernerThread.setDaemon(True)
    #execListernerThread.setDaemon(True)
    #poseListernerThread.start()
    #execListernerThread.start()
    wait_for_first_action()   
    #controllerPublisherThread.start()
    move()

