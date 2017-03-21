#!/usr/bin/env python

import rospy
from pioneer_hallway.msg import *
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from pioneer_hallway.srv import GetObsticles
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import *
import numpy as np


obsticles = []
world_map = None
position_estimate = None
map_resolution = None



#this is some factor of the map resolution
# used to correlate known objects to discovered ones
MAX_ACCEPTED_DIST_FACTOR = 0.5


class Obsticle:
    F = np.array( [[ 1, 0, .005, 0 ],
            [ 0, 1, 0, 0.005 ],
            [ 0, 0, 1, 0 ],
            [ 0, 0, 0, 1 ]])

    H = np.array([ 1, 1, 0, 0  ])
    def __init__(self, x, y, time, cov):
        print("Creating Obticle", x, y, time, cov)
        self.state = np.array([ x, y, 0, 0 ])
        self.cov = np.diag([cov, cov, cov, cov])
        self.lasttime = time
        self.health = 50

    def prediction(self, dt, state=None):
        if state == None:
            state = self.state
        #print("DT = " + str(dt))
        Obsticle.F[0][2] = dt
        Obsticle.F[1][3] = dt
        new_state = np.dot( Obsticle.F, state )
        new_cov = np.dot( Obsticle.F, np.dot( self.cov, Obsticle.F.T ) )
        return (new_state, new_cov)

    def update(self, state, time, cov):
        self.health = 50
        print("TIME %i" % time)
        (X_k, P_k) = self.prediction( time - self.lasttime )
        print("X_k: " + str(X_k))
        print("P_k: " + str(P_k))
        residual = state - np.dot( Obsticle.H, X_k )
        S_k = np.dot(Obsticle.H, np.dot( P_k, Obsticle.H.T )) + np.diag([ cov, cov, cov, cov ])
        K_k = np.dot( P_k, np.dot( Obsticle.H.T, np.linalg.inv( S_k ) ) )
        self.state = X_k + np.dot( K_k, residual )
        self.cov = P_k - np.dot( K_k, np.dot( S_k, K_k.T ) )

    def __str__(self):
        return str(self.state) + "  health: " + str(self.health)

    def dist( self, x, y, t):
        #print( "Time %i" % t )
        (state, _) = self.prediction(t - self.lasttime)
        return np.sqrt( ((x-state[0])**2) + ((y-state[1])**2 ))

    def nextState( self, x, y, t ):
        return np.array( [ x, y, (x - self.state[0]) / ( t - self.lasttime ), ( y - self.state[1] ) / ( t - self.lasttime ) ] )

    def age(self):
        self.health -= 1

    def is_healthy(self):
        return self.health > 0

    def generate_obsticle_prediction( self, dt, steps ):
        pred = ObsticlePrediction()
        pred.time = self.lasttime
        pred.timestep = dt
        pred.predictionCount = steps
        state = self.state
        cov = self.cov
        poses = []
        for i in range( steps ):
            pwc = PoseWithCovariance()
            pwc.covariance = np.zeros(36, dtype=np.float64)
            pwc.covarience[0] = cov[0][0]
            pwc.covarience[1] = cov[0][1]
            pwc.covarience[6] = cov[1][0]
            pwc.covarience[7] = cov[1][1]
            pwc.pose = Pose()
            pwc.pose.point = Point()
            pwc.pose.point.x = state[0]
            pwc.pose.point.y = state[1]
            pred.predictions.append( pwc )
            ( state, cov ) = self.prediction( dt, state=state )
        return pred

def distance(p1, p2):
    return np.sqrt( ( (p1[0]-p2[0]) ** 2 ) + ( (p1[1] - p2[1]) ** 2 ) )

def cluster_points(points, maxDistance):
    clusters = []
    usedPoints = np.zeros(len(points))

    for i in xrange(len( points )):
        if usedPoints[i] == 1:
            continue
        else:
            usedPoints[i] = 1
        c = (points[i][0], points[i][1])
        canopy = set([points[i]])
        for j in xrange(i+1, len( points ) ):
            if usedPoints[j] == 1:
                continue
            if distance( c, points[j] ) < maxDistance:
                canopy.add(points[j])
                c = ( c[0] * (1.0 - (1 / len(canopy))) , c[1] * (1.0 - (1 / len(canopy))))
                cx = points[j][0] / len( canopy )
                cy = points[j][1] / len( canopy )
                c = ( c[0] + cx, c[1] + cy )
                usedPoints[j] = 1
        clusters.append( canopy )
    return clusters





def subtract_map(points):
    global world_map, map_resolution
    '''
    Purpose: filters out points wich can be accounted for by the map
    Assumes: the points have been transformed into map space.
    '''
    return filter( lambda p: not ( np.floor(p[0] / map_resolution), np.floor( p[1] / map_resolution ) ) in world_map, points )

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
    return [ ( p[0] - by[0], p[1] - by[1] ) for p in points ]

def rotate_points_about_origin( points, theta ):
    '''
    Purpose: Rotate a set of points about the origin by 'theta' radians
    Assumes: theta is in radians
    '''
    R = np.array([ [np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)] ])
    return np.dot( points, R.T )

def update_position(data):
    global position_estimate
    position_estimate = data.pose.pose



def cluster_to_object( points ):
    sx = 0
    sy = 0
    for p in points:
        sx += p[0]
        sy += p[1]
    cx = sx / len(points)
    cy = sy / len(points)
    cov = 0
    for p in points:
        cov += ( p[0] - cx ) * ( p[1] - cy )
    return ( (cx, cy), cov / len(points) )



def handle_scan_input(data):
    global position_estimate
    global obsticles
    global map_resolution
    if position_estimate == None:
        print("No position esitmate")
        return
    time = float(data.header.stamp.secs) + (float(data.header.stamp.nsecs) / (1e-9))
    print(time)
    points = polar_to_euclidean( [ data.angle_min + (i * data.angle_increment)  for i in range(len(data.ranges))], data.ranges)

    t1 = 2 * ( (position_estimate.orientation.w * position_estimate.orientation.z) + ( position_estimate.orientation.x * position_estimate.orientation.y ) )
    t2 = 1 + ( 2 * ( (position_estimate.orientation.y ** 2) + (position_estimate.orientation.z ** 2)))
    theta = -np.arctan2( t1, t2 )
    print("Theta: %f" % theta)
    #points = np.dot( points, np.array( [[ -position_estimate.orientation.x, position_estimate.orientation.y ],
    #    [-position_estimate.orientation.y, -position_estimate.orientation.x]] ) )
    points = rotate_points_about_origin(points, theta)
    points = translate_points( points, ( position_estimate.position.x, position_estimate.position.y ) )
    points = subtract_map( points )
    clusters = cluster_points(points, MAX_ACCEPTED_DIST_FACTOR)
    found_obsticles = map( cluster_to_object, clusters )

    obsts_living = np.zeros( len(obsticles) )
    for ( c, cov ) in found_obsticles:
        closest = None
        minDist = None
        for i in range(len(obsticles)):
            if (minDist == None and obsticles[i].dist(c[0], c[1], time) < ( MAX_ACCEPTED_DIST_FACTOR * map_resolution )) or obsticles[i].dist( c[0], c[1], time) < minDist:
                minDist = obsticles[i].dist( c[0], c[1], time)
                closest = i
        if closest == None:
            obsticles.append( Obsticle(c[0], c[1], time, cov) )
        else:
            if i < len( obsts_living ):
                obsticles[i].update( obsticles[i].nextState( c[0], c[0], time ), time, cov )
                obsts_living[i] = 1
    for i, h in zip( range(len(obsts_living)), obsts_living ):
        if h == 0:
            obsticles[i].age()

    #obsticles = [ o for o in obsticles if o.is_healthy() ]
    print( '\n'.join( map( str, obsticles ) ) )


def handle_obsticle_request(req):
    global obsticles
    obs = Obsticles()
    obs.count = len(obsticles)
    for o in obsticles:
        obs.obsticles.append( o.generate_obsticle_prediction(req.dt * (10**9), req.steps) )
    return obs




def init_node():
    global world_map
    global map_resolution
    rospy.init_node('obst_tracker')
    rospy.wait_for_service('static_map')
    try:
        world_map = (rospy.ServiceProxy('static_map', GetMap)()).map
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
    rospy.Subscriber("/RosAria/lms5XX_1_laserscan", LaserScan, handle_scan_input)
    rospy.Subscriber("/RosAria/pose", Odometry, update_position)




if __name__ == '__main__':
    init_node()
    subscribe()
    rospy.spin()
