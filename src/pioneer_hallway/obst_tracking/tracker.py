#!/usr/bin/env python

import rospy
from pioneer_hallway.msg import *
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from nav_msgs.srv import GetMap
from pioneer_hallway.srv import GetObstacles
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import *
from std_msgs.msg import Header
import numpy as np
from tf import TransformListener, transformations


obsticles = []
world_map = None
position_estimate = None
map_resolution = None
map_pose = None
cloud_pub = None

transformer = None



#this is some factor of the map resolution
# used to correlate known objects to discovered ones
MAX_ACCEPTED_DIST_FACTOR = 0.5


class Obsticle:
    F = np.array( [[ 1, 0, .005, 0 ],
            [ 0, 1, 0, 0.005 ],
            [ 0, 0, 1, 0 ],
            [ 0, 0, 0, 1 ]])

    H = np.array([ 1, 0, 0, 0  ])
    def __init__(self, x, y, time, cov, radius):
        print("Creating Obticle", x, y, time, radius)
        self.state = np.array([ x, y, 0, 0 ])
        self.cov = np.diag([cov, cov, cov, cov])
        self.radius = radius
        self.lasttime = time
        self.health = 50

    def prediction(self, dt, state=None):
        if state == None:
            state = self.state
        #print("DT = " + str(dt))
        Obsticle.F[0][2] = dt
        Obsticle.F[1][3] = dt
        new_state = np.dot( Obsticle.F, state )
        # new_cov = np.dot( Obsticle.F, np.dot( self.cov, Obsticle.F.T ) )
        return new_state #(new_state, new_cov)

    def update(self, state, time, cov):
        self.health = 50
        # (X_k, P_k) = self.prediction( time - self.lasttime )
        # residual = state - np.dot( Obsticle.H, X_k )
        # S_k = np.dot(Obsticle.H, np.dot( P_k, Obsticle.H.T )) + np.eye(X_k.shape[0])
        #
        # K_k = np.dot( P_k, np.dot( Obsticle.H.T, np.linalg.inv( S_k ) ) )
        # self.state = X_k + np.dot( K_k, residual )
        # self.cov = P_k - np.dot( K_k, np.dot( S_k, K_k.T ) )
        # self.lasttime = time
        predicted_state = self.prediction( time - self.lasttime )
        self.state[0] = (state[0] + predicted_state[0])/2
        self.state[1] = (state[1] + predicted_state[1])/2
        self.state[2] = (state[2] + self.state[2])/2
        self.state[3] = (state[3] + self.state[3])/2
        self.lasttime = time



    def __str__(self):
        return str(self.state) + "  health: " + str(self.health)

    def dist( self, x, y, t):
        state = self.prediction(t - self.lasttime)
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
        for i in range( steps ):
            obstacle = Obstacle()
            obstacle.x = state[0]
            obstacle.y = state[1]
            obstacle.r = self.radius
            obstacle.time = self.lasttime + ( dt * i )
            obstacle.cov = (float(i)+1) / (float(steps)+1)
            pred.predictions.append( obstacle )
            state = self.prediction( dt, state=state )
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
                # c = ( c[0] * (1.0 - (1 / len(canopy))) , c[1] * (1.0 - (1 / len(canopy))))
                # cx = points[j][0] / len( canopy )
                # cy = points[j][1] / len( canopy )
                # c = ( c[0] + cx, c[1] + cy )
                usedPoints[j] = 1
        clusters.append( canopy )
    return clusters




def emitPointCloud(points):
    global cloud_pub
    head = Header()
    head.seq = 1
    head.stamp = rospy.Time.now()
    head.frame_id = "/map"
    points = [ Point32( x, y, 0.0 ) for x, y in points ]
    pc = PointCloud( head,  points, [ ChannelFloat32("", []) for _ in range(len(points))])
    cloud_pub.publish( pc )


def subtract_map(points):
    global world_map, map_resolution
    '''
    Purpose: filters out points wich can be accounted for by the map
    Assumes: the points have been transformed into map space.
    '''


    return filter( lambda p: not ( round(p[0] / map_resolution), round( p[1] / map_resolution ) ) in world_map and
        not ( round(p[0] / map_resolution)-1, round( p[1] / map_resolution ) ) in world_map and
        not ( round(p[0] / map_resolution)+1, round( p[1] / map_resolution ) ) in world_map and
        not ( round(p[0] / map_resolution)-1, round( p[1] / map_resolution )+1 ) in world_map and
        not ( round(p[0] / map_resolution)-1, round( p[1] / map_resolution )-1 ) in world_map and
        not ( round(p[0] / map_resolution)+1, round( p[1] / map_resolution )+1 ) in world_map and
        not ( round(p[0] / map_resolution)+1, round( p[1] / map_resolution )-1 ) in world_map and
        not ( round(p[0] / map_resolution), round( p[1] / map_resolution )+1 ) in world_map and
        not ( round(p[0] / map_resolution), round( p[1] / map_resolution )-1 ) in world_map and
        not ( round(p[0] / map_resolution)-2, round( p[1] / map_resolution ) ) in world_map and
        not ( round(p[0] / map_resolution)+2, round( p[1] / map_resolution ) ) in world_map and
        not ( round(p[0] / map_resolution)-2, round( p[1] / map_resolution )+2 ) in world_map and
        not ( round(p[0] / map_resolution)-2, round( p[1] / map_resolution )-2 ) in world_map and
        not ( round(p[0] / map_resolution), round( p[1] / map_resolution )+2 ) in world_map and
        not ( round(p[0] / map_resolution), round( p[1] / map_resolution )-2 ) in world_map, points )

def polar_to_euclidean( angles, ranges ):
    '''
    Purpose: converts an array of angles and ranges into x,y pairs in euclidean space
    Assumes: angles are in radians and the dimensions of both array are equal
    '''
    return [ [ np.cos(theta)*(r+.02), np.sin(theta)*r ] for theta, r in zip( angles, ranges ) ]

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
    cov = len(points)
    return ( (cx, cy), cov )



def handle_scan_input(data):
    global position_estimate, transformer
    global obsticles
    global map_resolution, world_map
    if position_estimate == None:
        print("No position esitmate")
        return
    time = float(data.header.stamp.secs) + (float(data.header.stamp.nsecs) / (1e9))
    step = (data.angle_max - data.angle_min) / float(len(data.ranges))
    points = polar_to_euclidean( [ data.angle_min + (i * step)  for i in range(len(data.ranges))], data.ranges)

    #points = np.dot( points, np.array( [[ -position_estimate.orientation.x, position_estimate.orientation.y ],
    #    [-position_estimate.orientation.y, -position_estimate.orientation.x]] ) )



    t = transformer.getLatestCommonTime("/base_link", "/laser_frame")
    pos, quart = transformer.lookupTransform("/laser_frame", "/base_link", t)
    t2 = transformer.getLatestCommonTime("/odom", "/base_link")
    pos2, quart2 = transformer.lookupTransform("/base_link", "/odom", t2)
    t3 = transformer.getLatestCommonTime("/odom", "/map")
    pos3, quart3 = transformer.lookupTransform("/odom", "/map", t3)

    angle = transformations.euler_from_quaternion( quart )[2]
    angle += transformations.euler_from_quaternion( quart2 )[2]
    angle += transformations.euler_from_quaternion( quart3 )[2]
    points = rotate_points_about_origin(points, angle)

    pos = (pos[0] + pos2[0] + pos3[0], pos[1] + pos2[1] + pos3[1])


    points = translate_points( points, ( pos[0], pos[1] ) )

    points = subtract_map( points )



    # emitPointCloud(list(world_map))
    clusters = cluster_points(points, .5)
    found_obsticles = map( cluster_to_object, clusters )
    #emitPointCloud(map(lambda x: x[0], found_obsticles))
    if len(obsticles) > 0:
         emitPointCloud(map( lambda x: (x.state[0], x.state[1]), obsticles))
    print("Time %f" % (time))
    obsts_living = np.zeros( len(obsticles) )
    for ( c, cov ) in found_obsticles:
        closest = None
        minDist = None
        for i in range(len(obsticles)):
            if (minDist == None and obsticles[i].dist(c[0], c[1], time) < ( .5 )) or obsticles[i].dist( c[0], c[1], time) < minDist:
                minDist = obsticles[i].dist( c[0], c[1], time)
                closest = i
            # else:
            #     print(obsticles[i].dist(c[0], c[1], time))
        if closest == None:
            obsticles.append( Obsticle(c[0], c[1], time, 0.1, cov) )
        else:
            if time > obsticles[closest].lasttime:
                obsticles[closest].update( obsticles[closest].nextState( c[0], c[1], time ), time, 0.1 )
                obsts_living[closest] = 1
    for i, h in zip( range(len(obsts_living)), obsts_living ):
        if h == 0:
            obsticles[i].age()

    obsticles = [ o for o in obsticles if o.is_healthy() ]
    print( '\n'.join( map( str, obsticles ) ) )


def handle_obsticle_request(req):
    global obsticles
    obs = Obstacles()
    obs.count = len(obsticles)
    for o in obsticles:
        obs.obstacles.append( o.generate_obsticle_prediction(req.dt, req.steps) )
    return obs




def init_node():
    global world_map, transformer
    global map_resolution, cloud_pub
    global map_pose
    rospy.init_node('obst_tracker')
    rospy.wait_for_service('static_map')
    try:
        world_map = (rospy.ServiceProxy('static_map', GetMap)()).map
        meta_map = world_map.info
        real_map = set()
        map_resolution = meta_map.resolution
        map_pose = meta_map.origin
        oX = map_pose.position.x / map_resolution
        oY = map_pose.position.y / map_resolution
        for x in xrange(meta_map.width):
            for y in xrange(meta_map.height):
                if world_map.data[ y*meta_map.width + x ] > 0.5:
                    print("Point in map (%f, %f)" % (round(x+oX), round(y+oY)))
                    real_map.add( (round(x+oX), round(y+oY)))

        print(map_pose)
        world_map = real_map
        cloud_pub = rospy.Publisher("non_map", PointCloud, queue_size=5)
        transformer = TransformListener()
    except rospy.ServiceException, e:
        print("Service Error: Could not obtain static map")

    rospy.Service("get_obstacles", GetObstacles, handle_obsticle_request)

def subscribe():
    rospy.Subscriber("/RosAria/lms5XX_1_laserscan", LaserScan, handle_scan_input)
    rospy.Subscriber("/RosAria/pose", Odometry, update_position)




if __name__ == '__main__':
    init_node()
    subscribe()
    rospy.spin()
