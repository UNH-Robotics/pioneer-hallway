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
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
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
primitives = read_primitives("../../../doc/motionPrimitive/primitives.txt")
#simulation_flag = sys.argv[2]

projected_pose = (0.0, 0.0, 0.0, 0.0)
predicted_pose = (0.0, 0.0, 0.0)
vel = 0.0
goal_is_set = True
planner_finished = True
first_iteration = True

''' 
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

def toEuler(orient):
  quaternion = (
    orient.x,
    orient.y,
    orient.z,
    orient.w)
  euler = tf.transformations.euler_from_quaternion(quaternion)
  return euler[2]

def poseCallBack(msg):
  global predicted_pose
  predicted_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, toEuler(msg.pose.pose.orientation))
 
def odomCallBack(msg):
  global vel
  vel = msg.twist.twist.linear.x 

'''
 Initialize our ROS nodes here -
        pub - our only publisher to give controller actions
        obst_track - assign our callback to the tracker
'''
rospy.init_node('executive', anonymous=False)
rate = rospy.Rate(4)
pub = rospy.Publisher('controller_msg', String, queue_size=1)
amcl_pose = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, poseCallBack)
plannerPathPub = rospy.Publisher('planner_snake', Path, queue_size=10)
plannerPathPub2 = rospy.Publisher('planner_snake2', Path, queue_size=10)
plannerPath = Path()
plannerPath2 = Path()
plannerPath.header.frame_id = "map"
plannerPath2.header.frame_id = "map"
odom = rospy.Subscriber('RosAria/pose', Odometry, odomCallBack)

'''
 gets the obstacles from 'get_obsticle' service
'''
def obstacles(dt, steps):
    try:
        get_obstacles_request = rospy.ServiceProxy('get_obstacles', GetObstacles)
        response = get_obstacles_request(dt, steps)
        print(str(response))
        return response
    except rospy.ServiceException, e:
        rospy.logerr("Service call has failed %s"%e)

def exec_control_pub(action):
  rospy.loginfo(action)
  pub.publish(action)

def update_cur(action):
    rospy.logdebug("update_cur action: " + str(action))
    planner_action = action[0]
    state = (str(predicted_pose[0]), str(predicted_pose[1]), str(vel), str(predicted_pose[2]))
    if planner_action == "":
      rospy.logfatal("NO ACTION RETURNED BY THE PLANNER, SHUTTING DOWN")
      exit()
    cur_primitive = primitives[planner_action]
    rospy.logdebug(str(cur_primitive.wa) + " " + str(cur_primitive.name) + " " + str(planner_action))
    rospy.logdebug(str(predicted_pose[0]) + " " + str(predicted_pose[1]) + " " + str(vel) + " " + str(predicted_pose[2]))
    project_pose = cur_primitive.apply(predicted_pose[0], predicted_pose[1], vel, 0, predicted_pose[2])
    global projected_pose
    projected_pose= (project_pose[0], project_pose[1], project_pose[3], project_pose[2]) 
    

def print_projected_pose(delimiter):
  return str(projected_pose[0]) + delimiter + str(projected_pose[1]) + delimiter \
     + str(projected_pose[2]) + delimiter + str(projected_pose[3])


def set_new_goal(p, nbsr, x, y):
  out = send_goal_to_planner(p, nbsr, x, y)
  while out == None:
    time.sleep(0.25)
    rospy.logwarn("waiting on planner to update goal")
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
    rospy.loginfo("from planner: " + out + "\n")
    if out == "READY":
      return out
    else:
      rospy.logwarn("planner needs more time to set up")
      return None
  except:
    rospy.logwarn("planner needs more time to set up")
    return None

def send_msg_to_planner(p, nbsr, t_time):
    if planner_finished:
      if not first_iteration:
        t_time = t_time + 245
      msg = "STATE\n" + str(t_time) + ' ' + print_projected_pose(" ") + '\n'
      for obst in ObstacleDb:
          msg = msg + "0 " + str(obst.x) + ' ' + str(obst.y) + ' ' + str(obst.r) + ' ' + str(obst.cov) + '\n'
      msg = msg + "END\n"
      rospy.loginfo("sending new state to plan to: " + msg)
      p.stdin.write(str.encode(msg))
      p.stdin.flush()
    return True

def check_planner_for_msg(p, nbsr):
    try:
        plan = []
        projection = []
        out = nbsr.readline(0.01)
        t_time = int(out.split(' ', 1)[1])
        rospy.logdebug("time_stamp_from_planner: " + str(t_time))
        rospy.logdebug(out)
        while out != "END":
          out = nbsr.readline(0.01)
          plan.append(out)  
        out = nbsr.readline(0.01)
        rospy.logdebug(out)
        while out != "END":
          out = nbsr.readline(0.1)
          projection.append(out)
        rospy.logdebug("plan: " + str(plan) + "\n") 
        rospy.logdebug("projection: " + str(projection) + "\n")
        if out == None:
          rospy.logfatal("PLANNER DID NOT RETURN WITHIN THE TIMEBOUND, SHUTTING DOWN")
          exit()
          return (None,t_time, projection, plan)
        else:
          rospy.logdebug(plan[0])
          if plan[0].split(' ',1)[0] == "END":
            rospy.logfatal("PLANNER DID NOT RETURN A PLAN, SHUTTING DOWN")
            exit()
          return (plan[0].split(' ',1),t_time, projection, plan)
    except Exception, e:
        rospy.logfatal("EXECUTIVE OR ROSMASTER HAS FAILED, SHUTTING DOWN")
        rospy.logerr("%s"%e)
        exit()
        return (None,t_time, projection, plan)

def publish_path(projection, plan):
  plannerPath = Path()
  plannerPath2 = Path()
  plannerPath.header.frame_id = "map"
  plannerPath2.header.frame_id = "map"

  for step in projection:
    if step.split(' ',1)[0] != "END":
      pose = PoseStamped()
      pose.pose.position.x = float(step.split(' ',5)[1])
      pose.pose.position.y = float(step.split(' ',5)[2])
      plannerPath.poses.append(pose)
      plannerPath.header.stamp = rospy.Time.now()
  plannerPathPub.publish(plannerPath)
  for step in plan:
    if step.split(' ', 1)[0] != "END":
      pose = PoseStamped()
      pose.pose.position.x = float(step.split(' ', 5)[1])
      pose.pose.position.y = float(step.split(' ', 5)[2])
      plannerPath2.poses.append(pose)
      plannerPath2.header.stamp = rospy.Time.now()
  plannerPathPub2.publish(plannerPath)
  

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
        
        x = 0
        x0 = predicted_pose[0]
        y=100
        y0 = predicted_pose[1]
        t = 0.25
        a = 0.3
        i = 1
        v = 0
        while (y > -12):
           # rospy.loginfo("ellasped time: " + str(time.time()-cur_clock))
            # send the msg to the planner store the time it took
            #cur_clock, action = send_msg_to_planner(master_clock, planner, nbsr)
          if v<0.75:
            y = y0 - 0.5 * a * math.pow(i * t, 2)
            v = a * i * t
            action = "a12," + str(x0) + "," +str(y)+ " ," + str(v) + ",-1.57,12312"
          else:
            y=y+v*t
          exec_control_pub(action)
          #update_cur(action)
          i += 1;
          print i
          time.sleep(t)

        # while (x < 4):
        #     # rospy.loginfo("ellasped time: " + str(time.time()-cur_clock))
        #     # send the msg to the planner store the time it took
        #     #cur_clock, action = send_msg_to_planner(master_clock, planner, nbsr)
        #     x = x + v * t
        #     action = "a13," + str(x) + "," + y+ "," + str(v) + ",0,12312"
        #     exec_control_pub(action)
        #     time.sleep(t)

        # mya = ["a14,2.0,-0.37," +str(v) + ",0.38,12312",
        #        "a14,2.3,-0.08," +str(v) + ",0.7,12312",
        #        "a14,2.49,0.2," +str(v) + ",1.0,12312",
        #        "a14,2.55,0.4," +str(v) + ",1.38,12312"]

        # i = 0
        # while (i < 4):
        #     exec_control_pub(mya[i])
        #     i += 1;
        #     print i
        #     time.sleep(t)

        # y = 0.4
        # while (y < 4):
        #     # rospy.loginfo("ellasped time: " + str(time.time()-cur_clock))
        #     # send the msg to the planner store the time it took
        #     #cur_clock, action = send_msg_to_planner(master_clock, planner, nbsr)
        #     y = y + v * t
        #     action = "a13," + "2.65," + str(y) +"," + str(v) + ",1.57,12312"
        #     exec_control_pub(action)
        #     time.sleep(t)
            
            
        #raise rospy.ROSException("ESTOP")
    except (rospy.ROSInterruptException, rospy.ROSException):
        rospy.logerr("ESTOP")
        pass
  
# if __name__ == '__main__':
#     # wait for services before starting up ...
#     rospy.loginfo("Waiting for services before starting up ...")
#     #rospy.wait_for_service('get_obstacles')
#     #ObstacleDb = obstacles(0.1, 1)
#     # fork and create a child subprocess of the planner
#     rospy.loginfo("Running Planner with 3s startup time...")
#     planner = Popen(["./planner.sh", simulation_flag],
#                                stdin=PIPE,
#                                stdout=PIPE,
#                                bufsize=1,
#                                universal_newlines=True) 
    
     
#     time.sleep(3)
#     nbsr = NBSR(planner.stdout)
#     # give time for the planner to initialize
#     # the master clock for the planner
#     cur_map_goal = (0, 0)
#     sim_map_goal = (3.89, -0.61)
#     kings_map_goal = (-64.5, -39.8)

#     if simulation_flag == "-simulator":
#       cur_map_goal = sim_map_goal
#     else:
#       cur_map_goal = kings_map_goal
    
#     set_new_goal(planner, nbsr, cur_map_goal[0], cur_map_goal[1]) 
#     cur_clock = time.time()
#     t_time = int(cur_clock * 1000)
#     projected_pose = (predicted_pose[0], predicted_pose[1], vel, predicted_pose[2])
#     rospy.loginfo("Executive online...")
#     try:
#         while (1.25 >= (time.time() - cur_clock)):
#             cur_clock = time.time() 
#             if first_iteration:
#               t_time = t_time + 250
#             #send the msg to the planner store the time it took
#             success = send_msg_to_planner(planner, nbsr, t_time)
#             time.sleep(0.245)
#             #check for the action to be in the queue
#             (action, t_time, projection, plan) = check_planner_for_msg(planner, nbsr)
#             update_cur(action)
#             rospy.logdebug("action_from_planner: " + action[0])
#             cont_msg = action[0] + "," + print_projected_pose(",") + "," + str(t_time) + "\n"
#             rospy.logdebug("msg_to_controller: " + cont_msg)
#             exec_control_pub(cont_msg)
#             publish_path(projection, plan)
#             rospy.logdebug(str(time.time() - cur_clock))
#             #t_time = t_time - 5 
#             first_iteration = False
#         raise rospy.ROSException("ESTOP")
#     except (rospy.ROSInterruptException, rospy.ROSException):
#       rospy.logerr("ESTOP - iteration took too long: " + str(time.time() - cur_clock))

  










