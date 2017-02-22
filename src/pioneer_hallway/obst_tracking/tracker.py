#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped


obst_publisher = None
world_map = None
position_estimate = None

def update_position(data):
    position_estimate = data

def handle_scan_input(data):
    obst_publisher.publish('')

def init_node():
    obst_publisher = rospy.Publisher('obsticles', String, queue_size=1)
    rospy.init_node('obst_tracker')
    rospy.wait_for_service('static_map')
    try:
        world_map = rospy.ServiceProxy('static_map', OccupancyGrid)()
    except rospy.ServiceException, e:
        print("Service Error: Could not obtain static map")

def subscribe():
    rospy.Subscriber("scan", LaserScan, handle_scan_input)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, update_position)




if __name__ == '__main__':
    init_node()
    subscribe()
    rospy.spin()
