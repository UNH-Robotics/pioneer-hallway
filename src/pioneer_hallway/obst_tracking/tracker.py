#!/usr/bin/env python

import rospy
from pioneer_hallway.msg import Obsticles
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np


obsticles = []
world_map = None
position_estimate = None
map_resolution = None



class Obsticle:
    F = np.array( [[ 1, 0, .005, 0 ],
            [ 0, 1, 0, 0.005 ],
            [ 0, 0, 1, 0 ],
            [ 0, 0, 0, 1 ])

    H = np.array([ 1, 1, 0, 0  ])
    def __init__(self, x, y, time, cov):
        self.state = np.array([ x, y, 0, 0 ])
        self.cov = np.diag([cov, cov, cov, cov])
        self.lasttime = time

    def prediction(self, dt, state=None):
        if state == None:
            state = self.state
        new_state = np.dot( Obsticle.F, state )
        new_cov = np.dot( Obsticle.F, np.dot( self.cov, Obsticle.F.T ) )
        return (new_state, new_cov)

    def update(state, time, cov):
        (X_k, P_k) = prediction( time - self.lasttime )
        residual = state - np.dot( Obsticle.H, X_k )
        S_k = np.dot(Obsticle.H, np.dot( P_k, Obsticle.H.T )) + np.diag([ cov, cov, cov, cov ])
        K_k = np.dot( P_k, np.dot( Obtsicle.H.T, np.inv( S_k ) ) )
        self.state = X_k + np.dot( K_k, residual )
        self.cov = P_k - np.dot( K_k, np.dot( S_k, K_k.T ) )





def distance(p1, p2):
    return np.sqrt( np.pow( p1[0]-p2[0], 2 ) + np.pow( p1[1] - p2[1], 2 ) )

def cluster_points(points, maxDistance):
    clusters = []
    usedPoints = np.zeros(len(points))

    for i in xrange(len( points )):
        if usedPoints[i] == 1:
            continue
        else:
            usedPoints[i] = 1
        canopy = set([points[i]])
        for j in xrange(i+1, len( points ) ):
            if usedPoints[j] == 1:
                continue
            if dist( points[i], points[j] ) < maxDistance:
                canopy.add(point[j])
                usedPoints[j] = 1
        clusters.append( canopy )
    return clusters





def subtract_map(points):
    '''
    Purpose: filters out points wich can be accounted for by the map
    Assumes: the points have been transformed into map space.
    '''
    return np.array( filter( lambda p: not ( np.floor(p[0] / map_resolution), np.floor( p[1] / map_resolution ) ) in world_map, points ) )

def polar_to_euclidean( angles, ranges ):
    '''
    Purpose: converts an array of angles and ranges into x,y pairs in euclidean space
    Assumes: angles are in radians and the dimensions of both array are equal
    '''
    return [ [ np.cos(theta)*r, np.sin(theta)*r ] for theta, r in zip( angles, ranges ) ]

def translate_points(points, by):
    '''
    Purpose: Translates iterable object of points and translates them by (dx, dy) tuple 'by'
    Assumes: points are in the format [ [x1, y1], [x2, y2], ...]
    '''
    return [ [ p[0] - by[0], p[1] - by[1] ] for p in points ]

def rotate_points_about_origin( points, theta ):
    '''
    Purpose: Rotate a set of points about the origin by 'theta' radians
    Assumes: theta is in radians
    '''
    R = np.array([ [np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)] ])
    return np.dot( points, R.T )

def update_position(data):
    position_estimate = data.pose.pose

def handle_scan_input(data):
    if position_estimate == None:
        return
    points = polar_to_euclidean( [ data.angle_min + (i * data.angle_increment)  for i in range(len(data.ranges))], data.ranges)
    points = np.dot( points, np.array( [[ -position_estimate.orientation.x, position_estimate.orientation.y ],
        [-position_estimate.orientation.y, -position_estimate.orientation.x]] ) )
    points = translate_points( points, ( data.position.x, data.position.y ) )
    clusters = cluster_points(points)


def handle_obsticle_request(req):




def init_node():
    rospy.init_node('obst_tracker')
    rospy.wait_for_service('static_map')
    try:
        world_map = rospy.ServiceProxy('static_map', OccupancyGrid)()
        meta_map = world_map.info
        real_map = set()
        map_resolution = meta_map.resolution
        for x in xrange(meta_map.width):
            for y in xrange(meta_map.height):
                if world_map.data[ x*meta_map.width + y ] > 0.5:
                    real_map.add( (x, y) )
        world_map = real_map

    except rospy.ServiceException, e:
        print("Service Error: Could not obtain static map")

    rospy.Service("get_obsticles", GetObsticles, handle_obsticle_request)

def subscribe():
    rospy.Subscriber("scan", LaserScan, handle_scan_input)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, update_position)




if __name__ == '__main__':
    init_node()
    subscribe()
    rospy.spin()
