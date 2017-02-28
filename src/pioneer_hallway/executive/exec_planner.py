#! /usr/bin/env python
from collections import namedtuple
import subprocess
import time
import sys
import rospy
from std_msgs.msg import String
from nbstreamreader import NonBlockingStreamReader as NBSR

# set up a Db of structs for obstacles
# these are what we pass to the planner
Obstacle = namedtuple("Obstacle", "x y")
ObstacleDb = []

# callbacks
def obstacle_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "obstacle_msg %s", data.data)

# initialize our ROS nodes here
pub = rospy.Publisher('controller_msg', String, queue_size=1)
rospy.init_node('executive', anonymous=True)
rate = rospy.Rate(10)
rospy.Subscriber('obst_tracker', String, obstacle_callback)

def exec_control_pub(action):
    rospy.loginfo(action)
    pub.publish(action)

# instance variables used to keep track of current state
# update these when we do forward projection
cur_x = 1.0
cur_y = 1.0
cur_lin = 0.0
cur_rot = 0.0

def print_cur_state():
    return str(cur_x) + ' ' + str(cur_y) + ' ' + str(cur_lin) + ' ' + str(cur_rot)

def send_msg_to_planner(master_clock, p, nbsr):
    msg = str(int(time.time() - master_clock)) + '\n' + print_cur_state() + '\n'
    for obst in ObstacleDb:
        msg = msg + str(obst.x) + ' ' + str(obst.y) + '\n'
    msg = msg + "END\n"
    p.stdin.write(msg)
    # have to wait for the process to respond
    try:
        return (time.time(), nbsr.readline(0.350))
    except:
        return (time.time(), "")

if __name__ == '__main__':
    # fork and create a child subprocess of the planner
    print("Running Planner with 3s startup time...")
    planner = subprocess.Popen("./planner.sh -timeout 250", shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    nbsr = NBSR(planner.stdout)
    # give time for the planner to initialize
    time.sleep(3)
    # the master clock for the planner
    master_clock = time.time()
    cur_clock = time.time()
    try:
        while ((time.time() - cur_clock) < 1.0):
            print("ellasped time: " + str(time.time()-cur_clock))
            # send the msg to the planner store the time it took
            cur_clock, action = send_msg_to_planner(master_clock, planner, nbsr)
            exec_control_pub(action)
            master_clock = cur_clock
    except rospy.ROSInterruptException:
        print("ESTOP")
        pass
#    obstacle_listener()
#    try:        
#        send_msg_to_planner()
#        publish_action()
#    except rospy.ROSInterruptException:
#        pass
    
