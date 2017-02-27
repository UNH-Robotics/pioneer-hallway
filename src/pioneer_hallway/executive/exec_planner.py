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

def exec_control_pub(action):
    pub = rospy.Publisher('controller_msg', String, queue_size=1)
    rospy.init_node('exec_control_pub', anonymous=True)
    print("exec_control_pub sending action to controller...")
    rate = rospy.Rate(10)
    rospy.loginfo(action)
    pub.publish(action)
    rate.sleep()

# TODO: find out the obst type and fill in Db
def obstacle_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "obstacle_msg: %s", data.data)

def obstacle_listener():
    rospy.init_node('exec_obs_listen', anonymous=True)
    rospy.Subscriber('obst_tracker', String, obstacle_callback)
    print("exec_obs_listen waiting for obstacles...")
    rospy.spin()

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
    print("SENT TO PLANNER:\n" + msg)
    action = ""
    p.stdin.write(msg)
    time.sleep(0.3)
    try:
        return (time.time(), nbsr.readline(0.3))
    except:
        return (time.time(), "")

if __name__ == '__main__':
    # fork and create a child subprocess of the planner
    planner = subprocess.Popen("./planner.sh -timeout 300", shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    nbsr = NBSR(planner.stdout)
    # give time for the planner to initialize
    time.sleep(3)
    # the master clock for the planner
    master_clock = time.time()
    try:
        while (master_clock - time.time() < 300):
            # send the msg to the planner store the time it took
            master_clock, action = send_msg_to_planner(master_clock, planner, nbsr)
            print(action)
            print("cur_state: " + print_cur_state())
            exec_control_pub(action)
            master_clock = time.time()
    except rospy.ROSInterruptException:
        print("ESTOP")
        pass
#    obstacle_listener()
#    try:        
#        send_msg_to_planner()
#        publish_action()
#    except rospy.ROSInterruptException:
#        pass
    
