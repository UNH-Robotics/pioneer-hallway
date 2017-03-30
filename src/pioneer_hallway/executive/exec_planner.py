#! /usr/bin/env python
from collections import namedtuple
from subprocess import Popen, PIPE, check_call
import time
import re
import sys
import rospy
import math
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nbstreamreader import NonBlockingStreamReader as NBSR
sys.path.insert(0, "../../../doc/motionPrimitive/")
from primutils import Primitive, read_primitives
from pioneer_hallway.srv import *

'''
 used to keep track of current state
 update these when we do forward projection 
 and/or when we look up the new state
'''

'''
 Read the primitives from the file
 store the change in velocity as tuples
 for easy calculatioin when keeping track
 of the current state
 TODO: read these from the file given 
       remove the hardcoded values
'''
primitives = read_primitives(sys.argv[1])
simulation_flag = sys.argv[2]

planner_return_state = (0.0, 0.0, 0.0, 0.0)

poseX = 0.0
poseY = 0.0
poseHeading = 0.0
vel = 0.0
goal_is_set = True
planner_finished = True

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

def poseCallBack(data):
  rospy.loginfo("updating pose from acml")
  pose = data
  global poseX 
  poseX = pose.pose.pose.position.x 
  global poseY 
  poseY = pose.pose.pose.position.y
  global poseHeading 
  poseHeading = toEuler(pose.pose.pose.orientation)
  rospy.logwarn("poseCallBack: " + str(poseX) + " " + str(poseY) + " " + str(poseHeading))
  #rospy.loginfo(pose)

def velCallBack(data):
  rospy.logwarn("updating velocities from cmd_vel " + print_cur_state())
  global vel
  vel = data.linear.x
  
 
'''
 Initialize our ROS nodes here -
        pub - our only publisher to give controller actions
        obst_track - assign our callback to the tracker
'''
rospy.init_node('executive', anonymous=True)
rate = rospy.Rate(60)
pub = rospy.Publisher('controller_msg', String, queue_size=1)
obst_tracker = rospy.ServiceProxy('get_obsticles', GetObsticles)
amcl_pose = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, poseCallBack)
cmd_vel = rospy.Subscriber('cmd_vel', Twist, velCallBack)

def toEuler(orient):
  quaternion = (
    orient.x,
    orient.y,
    orient.z,
    orient.w)
  euler = tf.transformations.euler_from_quaternion(quaternion)
  if euler[0] == 0 and euler[1] == 0:
    return euler[2]
  return None

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
    rospy.loginfo("ecp: " + action)
    pub.publish(action)

def update_lcur(msg):
  #rospy.loginfo("update_cur_msg: " + str(len(msg)))
  if len(msg) == 1:
    rospy.logwarn("planner reporting we're at the goal")
  else:
    state = msg[1].rstrip().split(' ',4)
    print(state)
    global poseX
    global poseY
    global vel
    global poseHeading
    poseX = state[0]
    poseY = state[1]
    vel = state[2]
    poseHeading = state[3]
    #rospy.loginfo("new updated lstate: " + print_cur_state())

def update_cur(action):
    rospy.loginfo("starting update_cur: " + print_cur_state())
    str_index = action[0]
    state = (str(poseX), str(poseY), str(vel), str(poseHeading))
    if len(action) > 1:
      state = action[1].rstrip().split(' ', 4)
    rospy.loginfo("updating state with action: " + str_index)
    index_action = int(str_index[1:])
    cur_primitive = primitives[index_action]
    rospy.loginfo("update_cur: " + str(state[0]) + " " + str(state[1]) + " " + str(state[2]) + " " + str(state[3]))
    global planner_return_state
    planner_return_state = (state[0], state[1], state[2], state[3]) 
    rospy.loginfo("new updated state: " + str(planner_return_state[0]) + " " + str(planner_return_state[1]) + " " + str(planner_return_state[2]) + " " + str(planner_return_state[3]))
    
def print_cur_state():
    return str(poseX) + ' ' + str(poseY) + \
        ' ' + str(vel) + ' ' + str(poseHeading)

    
def print_cur_cstate():
    return str(poseX) + ',' + str(poseY) + \
        ',' + str(vel) + ',' + str(poseHeading)


def set_new_goal(p, nbsr, x, y):
  out = send_goal_to_planner(p, nbsr, x, y)
  while out == None:
    time.sleep(0.25)
    rospy.logwarn("waiting on planner to update goal")
    out = nbsr.readline(0.25)
    out = nbsr.readline(0.25)
  rospy.loginfo("planner fired up and ready to go")
  global goal_is_set
  goal_is_set = True

def send_goal_to_planner(p, nbsr, x, y):
  goal_is_set = False
  msg = "GOAL\n" + str(x) + " " + str(y) + "\n"
  rospy.loginfo("sending new goal to plan towards: " + msg)
  p.stdin.write(msg)
  try:
    out = nbsr.readline(0.20)
    out = nbsr.readline(0.20)
    rospy.loginfo("from planner: " + out + "\n")
    if out == "READY":
      return out
    else:
      rospy.logwarn("planner needs more time to set up")
      return None
  except:
    rospy.logwarn("planner needs more time to set up")
    return None

def send_msg_to_planner(p, nbsr):
    if planner_finished:
      msg = "STATE\n" + str(int(time.time() * 1000) + 250) + ' ' + print_cur_state() + '\r' 
      for obst in ObstacleDb:
          msg = msg + str(obst.x) + ' ' + str(obst.y) + '\r'
      msg = msg + "END\n"
      rospy.loginfo("sending new state to plan to: " + msg)
      p.stdin.write(msg)
      p.stdin.flush()
    # time.sleep(0.25)
    # have to wait for the process to respond
    time.sleep(0.1)
    try:
        (t,a) = (time.time(), nbsr.readline(0.05))
        (t2,b) = (t, nbsr.readline(0.025))
        if a == None:
          (t,a) = (time.time(), nbsr.readline(0.05))
          (t2,b) = (t, nbsr.readline(0.025))
        rospy.loginfo("plan msg: " + a + "\n") 
        rospy.loginfo("plan action: " + b + "\n")
        if a == None:
          global planner_finished
          planner_finished = False
          rospy.logerr("planner did not respond assuming no action - planner finished? " + str(planner_finished))
          return (t, ["a7"])
        else:
          global planner_finished
          planner_finished = True
          return (t,b.split(' ',1))
    except:
        global planner_finished
        planner_finished = False
        rospy.logerr("planner did not respond assuming no action - planner finished? " + str(planner_finished))
        return (time.time(), ["a7"])

if __name__ == '__main__':
    # wait for services before starting up ...
    rospy.loginfo("Waiting for services before starting up ...")
    #rospy.wait_for_service('get_obsticles')
    # fork and create a child subprocess of the planner
    rospy.loginfo("Running Planner with 3s startup time...")
    planner = Popen(["./planner.sh", simulation_flag],
                               stdin=PIPE,
                               stdout=PIPE,
                               bufsize=1,
                               universal_newlines=True) 
    
     
    time.sleep(3)
    nbsr = NBSR(planner.stdout)
    # give time for the planner to initialize
    # the master clock for the planner
    cur_map_goal = (0, 0)
    sim_map_goal = (0.02, -0.421)
    kings_map_goal = (-64.5, -39.8)

    if simulation_flag == "-simulator":
      cur_map_goal = sim_map_goal
    else:
      cur_map_goal = kings_map_goal
    
    set_new_goal(planner, nbsr, cur_map_goal[0], cur_map_goal[1]) 
    master_clock = time.time()
    cur_clock = time.time()
    try:
        while ((time.time() - cur_clock) < 1.0):
            #send the msg to the planner store the time it took
            cur_clock, action = send_msg_to_planner(planner, nbsr)
            time.sleep(0.05)
            update_cur(action)
            cont_msg = action[0] + "," + str(planner_return_state[0]) + "," + str(planner_return_state[1]) + "," + str(planner_return_state[2]) + "," + str(planner_return_state[3]) + "," + str(int(time.time() * 1000) + 250) + "\n"
            exec_control_pub(cont_msg)
            #rospy.loginfo("ellasped time: " + str(time.time()-cur_clock))
            #time.sleep((time.time() - cur_clock))
            master_clock = cur_clock
            #time.sleep(0.20)
            rospy.logerr(str(time.time() - cur_clock))
        raise rospy.ROSException("ESTOP")
    except (rospy.ROSInterruptException, rospy.ROSException):
      rospy.logerr("ESTOP - iteration took too long: " + str(time.time() - cur_clock))










