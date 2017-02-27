#! /usr/bin/env python
from collections import namedtuple
import subprocess
import time
import rospy
from std_msgs.msg import String

# set up a Db of structs for obstacles
# these are what we pass to the planner
Obstacle = namedtuple("Obstacle", "x y")
ObstacleDb = []

# TODO: find out the obst type and fill in Db
def obstacle_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "obstacle_msg: %s", data.data)

def obstacle_listener():
    rospy.init_node('exec_obs_listenp', anonymous=True)
    rospy.Subscriber('obst_tracker', String, obstacle_callback)
    print "exec_obs_listenp set up and spinning..."
    rospy.spin()

# instance variables used to keep track of current state
# update these when we do forward projection
cur_x = 1.0
cur_y = 1.0
cur_lin = 0.0
cur_rot = 0.0

def print_cur_state():
    return str(cur_x) + ' ' + str(cur_y) + ' ' + str(cur_lin) + ' ' + str(cur_rot)

def send_msg_to_planner(master_clock, p):
    msg = str(int(time.time() - master_clock)) + '\n' + print_cur_state() + '\n'
    for obst in ObstacleDb:
        msg = msg + str(obst.x) + ' ' + str(obst.y) + '\n'
    msg = msg + "END"
    print("SENT TO PLANNER:\n" + msg)
    p.stdin.write(msg)
    return time.time()

if __name__ == '__main__':
    # fork and create a child subprocess of the planner
    planner = subprocess.Popen("./planner.sh", shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    # the master clock for the planner
    master_clock = time.time()
    # send the msg to the planner store the time it took
    master_clock = send_msg_to_planner(master_clock, planner)
    action = planner.stdout.readline()
    print(action)
#    obstacle_listener()
#    try:        
#        send_msg_to_planner()
#        publish_action()
#    except rospy.ROSInterruptException:
#        pass
    
