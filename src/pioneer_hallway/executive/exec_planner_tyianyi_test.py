#! /usr/bin/env python
from collections import namedtuple
import subprocess
import time
import sys
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nbstreamreader import NonBlockingStreamReader as NBSR
sys.path.insert(0, "../../../doc/motionPrimitive/")
sys.path.insert(0, "../src/")
#import Lib
from primutils import Primitive, read_primitives
from pioneer_hallway.srv import *

pi = math.pi
ad = 0.25
'''
 Read the primitives from the file
 store the change in velocity as tuples
 for easy calculatioin when keeping track
 of the current state
 TODO: read these from the file given 
       remove the hardcoded values
'''
primitives = read_primitives("../../../doc/motionPrimitive/primitives.txt")
primitive_tuples = [(0.075,ad*pi/4),(0,ad*pi/4),(-0.075,ad*pi/4),(0.075,ad*pi/8),(0,ad*pi/8),(-0.075,ad*pi/8),
                    (0.075,0),(0,0),(-0.075,0),(0.075,ad*-1*pi/4),(0,ad*-1*pi/4),(-0.075,ad*-1*pi/4),
                    (0.075,ad*-1*pi/8),(0,ad*-1*pi/8),(-0.075,ad*-1*pi/8)]


''' 
 TODO: find out the msg type and read
       in the obstacles into the Db
 set up a Db of structs for obstacles
 these are what we pass to the planner

 t = obst.predictions[i].header.stamp.sec
 x = obst.predictions[i].pose.pose.position.x
 y = obst.predictions[i].pose.pose.position.y
 cov = obst.predictions[i].pose.covariance
       6x6 covariance matrix
       (x, y, z, rotX, rotY, rotZ)
       we care about covX, covY
       covX = cov[?]
       covY = cov[?]
'''
Obstacle = namedtuple("Obstacle", "t x y cov")
ObstacleDb = []

'''
 Initialize our ROS nodes here -
        pub - our only publisher to give controller actions
        obst_track - assign our callback to the tracker
'''
rospy.init_node('executive', anonymous=True)
rate = rospy.Rate(10)
pub = rospy.Publisher('controller_msg', String, queue_size=1)
obst_tracker = rospy.ServiceProxy('get_obsticles', GetObsticles)

'''
 gets the obstacles from 'get_obsticle' service
'''
def obstacles(dt, steps):
    try:
        response = obst_tracker(dt, steps)
        return response
    except rospy.ServiceException, e:
        rospy.logerr("Service call has failed %s"%e)


def exec_control_pub(action):
    rospy.loginfo(action)
    pub.publish(action)

'''
 used to keep track of current state
 update these when we do forward projection 
 and/or when we look up the new state
'''
CurrentState = namedtuple("CurrentState", "x y lin rot head")
CurrentState.x = 1.0
CurrentState.y = 1.0
CurrentState.lin = 0.0
CurrentState.rot = 0.0
CurrentState.head = 0.0

'''
 Sets the CurrentState to a new given state
 pos (x,y) linear rotational velocity
'''
def set_cur(x, y, lin, rot, head):
    CurrentState.x = x
    CurrentState.y = y
    CurrentState.lin = lin
    CurrentState.rot = rot
    CurrentState.head = head

def update_cur(action):
    str_index = action.rstrip()
    index_action = int(str_index[1:])
    (add_lin, add_rot) = primitive_tuples[index_action]
    cur_primitive = primitives[index_action]
    if cur_primitive.get_entry(CurrentState.lin, CurrentState.rot, CurrentState.head) == None:
        newState = cur_primitive.apply(CurrentState.x, CurrentState.y, CurrentState.lin, CurrentState.rot, CurrentState.head)
        set_cur(newState[0], newState[1], CurrentState.lin+add_lin, CurrentState.rot+add_rot, newState[2])
    else:
        newState = cur_primitive.get_entry(CurrentState.lin, CurrentState.rot, CurrentState.head)
        set_cur(newState[0], newState[1], CurrentState.lin+add_lin, CurrentState.rot+add_rot, newState[2])
    rospy.loginfo(print_cur_state())
    
def print_cur_state():
    return str(CurrentState.x) + ' ' + str(CurrentState.y) + \
        ' ' + str(CurrentState.lin) + ' ' + str(CurrentState.rot) + ' ' + str(CurrentState.head)


def send_msg_to_planner(master_clock, p, nbsr):
    msg = str(int(time.time() - master_clock)) + '\n' + print_cur_state() + '\n'
    for obst in ObstacleDb:
        msg = msg + str(obst.x) + ' ' + str(obst.y) + '\n'
    msg = msg + "END\n"
    p.stdin.write(msg)
    time.sleep(0.25)
    # have to wait for the process to respond
    try:
        return (time.time(), nbsr.readline(0.25))
    except:
        return (time.time(), "")

if __name__ == '__main__':
    # # wait for services before starting up ...
    # rospy.loginfo("Waiting for services before starting up ...")
    # rospy.wait_for_service('get_obsticles')
    # # fork and create a child subprocess of the planner
    # rospy.loginfo("Running Planner with 3s startup time...")
    # planner = subprocess.Popen("./planner.sh -timeout 250",
    #                            shell=True,
    #                            stdin=subprocess.PIPE,
    #                            stdout=subprocess.PIPE)
    # nbsr = NBSR(planner.stdout)
    # give time for the planner to initialize
    time.sleep(3)
    # the master clock for the planner
    master_clock = time.time()
    cur_clock = time.time()
    try:
        mya = ["a22,-1.76,-0.5,0.1,0,12312",
               "a22,-1.41,-0.5,0.2,0,12312",
               "a22,-1.41,-0.5,0.2,0,12312",
               "a22,-0.07,-0.5,0.3,0,12312",
               "a22,0.1,-0.5,0.4,0,12312",
               "a22,0.6,-0.5,0.5,0,12312",
               "a22,1.0,-0.5,0.6,0,12312",
               "a22,1.2,-0.5,0.7,0,12312",
               "a22,1.5,-0.5,0.8,0,12312",
               "a22,1.9,-0.5,0.9,0,12312",
               "a22,3.0,-0.5,1.0,0,12312"]
        
        x = -2
        t = 0.5
        a = 0.3
        i = 1
        v = 0
        while (x < 0.2):
           # rospy.loginfo("ellasped time: " + str(time.time()-cur_clock))
            # send the msg to the planner store the time it took
            #cur_clock, action = send_msg_to_planner(master_clock, planner, nbsr)
            x = -2 + 0.5 * a * math.pow(i * t, 2)
            v = a * i * t
            action = "a22," + "%.2f" % x + ",-0.5," +"%.2f" % v + ",0,12312"
            exec_control_pub(action)
            #update_cur(action)
            i += 1;
            print i
            time.sleep(t)

        while (x < 1.8):
            # rospy.loginfo("ellasped time: " + str(time.time()-cur_clock))
            # send the msg to the planner store the time it took
            #cur_clock, action = send_msg_to_planner(master_clock, planner, nbsr)
            x = x + v * t
            action = "a13," + "%.2f" % x + ",-0.5," +"%.2f" % v + ",0,12312"
            exec_control_pub(action)
            time.sleep(t)

        mya = ["a14,1.89,-0.33," +"%.2f" % v + ",0.38,12312",
               "a14,2.0,-0.27," +"%.2f" % v + ",0.7,12312",
               "a14,2.3,-0.08," +"%.2f" % v + ",1.0,12312",
               "a14,2.49,0.2," +"%.2f" % v + ",1.38,12312"]

        i = 0
        while (i < 4):
            exec_control_pub(mya[i])
            i += 1;
            print i
            time.sleep(t)

        y = 0.2
        while (y < 4):
            # rospy.loginfo("ellasped time: " + str(time.time()-cur_clock))
            # send the msg to the planner store the time it took
            #cur_clock, action = send_msg_to_planner(master_clock, planner, nbsr)
            y = y + v * t
            action = "a13," + "2.55," + "%.2f" % y +",%.2f" % v + ",1.57,12312"
            exec_control_pub(action)
            time.sleep(t)
            
            
        #raise rospy.ROSException("ESTOP")
    except (rospy.ROSInterruptException, rospy.ROSException):
        rospy.logerr("ESTOP")
        pass
