#!/usr/bin/python

import rospy
from pioneer_hallway.msg import *
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from nav_msgs.srv import GetMap
from pioneer_hallway.srv import GetObstacles
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import *
from std_msgs.msg import Header
import numpy as np
from tf import TransformListener, transformations
import time as ttime


obsticles = []
world_map = None
position_estimate = None
map_resolution = None
map_pose = None
cloud_pub = None
path_pub = None
transformer = None
sphere_pub = None



#this is some factor of the map resolution
# used to correlate known objects to discovered ones
MAX_ACCEPTED_DIST_FACTOR = 0.5



class Obsticle:
    A = np.array( [[ 1, 0, .005, 0 ],
            [ 0, 1, 0, 0.005 ],
            [ 0, 0, 1, 0 ],
            [ 0, 0, 0, 1 ]])
    H = np.array( [ [1, 0, 0, 0], [0, 1, 0, 0] ] )
    R = np.eye(2)

    lognorm = 1.0 / np.log(2)

    H = np.array([ 1, 0, 0, 0  ])
    def __init__(self, x, y, time, radius):
        #print("Creating Obticle", x, y, time, radius)
        self.state = np.array([ [x], [y], [0], [0] ])
        self.cov = np.eye(4) / 4
        self.Q = np.eye(4)
        self.radius = radius
        self.lasttime = time
        self.health = 5
        self.lifetime = 1.0
        Obsticle.H = np.array( [ [1, 0, 0, 0], [0, 1, 0, 0] ] )

    def prediction(self, dt, state=None, cov=None):
        if state == None:
            state = self.state
        if cov == None:
            cov = self.cov
        #print("DT = " + str(dt))

        Obsticle.A[0][2] = dt
        Obsticle.A[1][3] = dt
        new_state = np.dot( Obsticle.A, state )
        next_cov = np.dot(Obsticle.A, np.dot(cov, Obsticle.A.T)) + self.Q
        return (new_state, next_cov)

    def update(self, state, time, radius):
        self.health = 5
        self.lifetime += 1.0
        self.Q = np.eye(4) / (self.lifetime * 1.1 )
        self.radius = max( self.radius, radius )
        dt = time - self.lasttime

        Obsticle.A[0][2] = dt
        Obsticle.A[1][3] = dt


        self.state, self.cov = self.prediction( time - self.lasttime )
        # print("Given " + str(state) + " dt:  " + str( time - self.lasttime ))
        # print("Predict obst to state " + str(self.state) + "  age: " + str(self.lifetime) + "  H " + str(Obsticle.H))
        IM = np.dot(Obsticle.H, self.state)
        # print("IM: " + str( IM ))
        IS = Obsticle.R + np.dot(Obsticle.H, np.dot(self.cov, Obsticle.H.T))
        # print("IS: " + str(IS))
        K = np.dot(self.cov, np.dot(Obsticle.H.T, np.linalg.inv(IS)))
        # print("Kalman Gain: " + str(K))
        self.state = self.state + np.dot(K, (state-IM))
        self.cov = self.cov - np.dot(K, np.dot(IS, K.T))
        self.lasttime = time
        # print("updated obst to state " + str(self.state) + "  age: " + str(self.lifetime) + "\n\n")



    def __str__(self):
        return str(self.state) + "  health: " + str(self.health)

    def dist( self, x, y, t):
        state, cov = self.prediction(t - self.lasttime)
        theta = np.arctan2( x - state[0], y - state[1] )
        nx = (np.cos(theta) * self.radius) + state[0]
        ny = (np.sin(theta) * self.radius) + state[1]
        var = np.sqrt( cov[0][0] * cov[2][2] )
        #var = cov[0][0]
        px = (1.0/np.sqrt( 2 * np.pi * var )) * np.exp(-( ((x-nx)**2) / (2 * var) ))
        py = (1.0/np.sqrt( 2 * np.pi * var )) * np.exp(-( ((y-ny)**2) / (2 * var) ))

        # px = (1.0/np.sqrt( 2 * np.pi * cov[0][0] )) * np.exp(-( ((x-state[0])**2) / (2 * cov[0][0]) ))
        # py = (1.0/np.sqrt( 2 * np.pi * cov[0][0] )) * np.exp(-( ((y-state[1])**2) / (2 * cov[0][0]) ))
        # print(px[0], py[0], (px[0]*py[0]))
        return 1 - (px[0] * py[0])
        # return np.sqrt( ((x-state[0])**2) + ((y-state[1])**2 ))

    def nextState( self, x, y, t ):
        return [[x], [y]]
        # return np.array( [ [x], [y], [(x - self.state[0]) / ( t - self.lasttime )], [( y - self.state[1] ) / ( t - self.lasttime )] ] )

    def age(self):
        self.health -= 1

    def is_healthy(self):
        return self.health > 0 and np.sqrt( self.state[2]**2 + self.state[3]**2 ) < 2.5

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
            obstacle.r = self.radius if self.radius > 0 else 0.1
            obstacle.time = self.lasttime + ( dt * i )
            obstacle.cov = np.sqrt(cov[0][0] * cov[2][2])
            # print(obstacle.cov)
            pred.predictions.append( obstacle )
            state, cov = self.prediction( dt, state=state, cov=cov )
            # if obstacle.cov < (10 * self.radius):
            #     state, cov = self.prediction( dt, state=state, cov=cov )
            # else:
            #     state, _ = self.prediction( dt, state=state, cov=cov )
        print(pred.predictions[-1].cov)

        # print "-----------------------------------------------------------------------------"
        return pred

    def makeMarker( self, dt, steps ):
        pred = self.generate_obsticle_prediction( dt, steps )
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obst_tracker"
        marker.action = Marker.ADD
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.b = 1.0

        sphereList = list()

        marker.points = list()

        covMax = max( o.cov for o in pred.predictions )

        for o in pred.predictions:
            sphere = Marker()
            sphere.header.frame_id = "/map"
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "obst_tracker"
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            p = Point()
            p.z = 0
            p.x = o.x
            p.y = o.y
            sphere.pose.position.x = o.x
            sphere.pose.position.y = o.y
            sphere.pose.position.z = 0
            #print(o.r)
            sphere.scale.x = sphere.scale.y = o.r * 2
            sphere.scale.z=  o.r * 2
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 1.0
            sphere.color.a = min(1, (o.r**2) / ((o.cov/2)**2) )
            marker.points.append( p )
            sphereList.append(sphere)
        sphereList = sphereList[0::2]
        return marker, sphereList

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
    position_estimate = data.pose.pose.position



def cluster_to_object( points ):
    sx = 0
    sy = 0
    maxDist = 0
    for p in points:
        sx += p[0]
        sy += p[1]
        for p2 in points:
            maxDist = max(maxDist, distance(p, p2))
    cx = sx / len(points)
    cy = sy / len(points)
    return (cx, cy), maxDist/2

def pubPath( obsticles ):
    global path_pub, sphere_pub

    markerArray = MarkerArray()
    marker2 = MarkerArray()
    i = 0
    for m, l in map( lambda o: o.makeMarker( 0.25, 20 ), obsticles ):
        m.id = i
        j = i
        for s in l:
            s.id = j
            j+=1
            marker2.markers.append(s)
        i = j + 1
        markerArray.markers.append(m)
    path_pub.publish(markerArray)
    sphere_pub.publish( marker2 )

def handle_scan_input(data):
    global position_estimate, transformer
    global obsticles
    global map_resolution, world_map
    # tStart = ttime.time()
    time = float(data.header.stamp.secs) + (float(data.header.stamp.nsecs) / (1e9))
    step = (data.angle_max - data.angle_min) / float(len(data.ranges))
    points = polar_to_euclidean( [ data.angle_min + (i * step)  for i in range(len(data.ranges)) if data.ranges[i] < 7 ], [d for d in data.ranges if d < 7])


    t = transformer.getLatestCommonTime("laser_frame", "/map")
    pos, quart = transformer.lookupTransform("/map", "laser_frame", t)

    angle = transformations.euler_from_quaternion( quart )[2]
    points = rotate_points_about_origin(points, angle)


    points = translate_points( points, ( -pos[0], -pos[1] ) )

    points = subtract_map( points )

    location = ( pos[0], pos[1] )
    # print("Location: " + str(location))

    #emitPointCloud(map( lambda x: ( x[0]*map_resolution, x[1]*map_resolution ), list(world_map)))
    clusters = cluster_points(points, .5)
    found_obsticles = map( cluster_to_object, clusters )
    #emitPointCloud(map(lambda x: x[0], found_obsticles))
    if len(obsticles) > 0:
          emitPointCloud(map( lambda x: (x.state[0], x.state[1]), obsticles))
          pubPath( obsticles )
          print("obst count = " + str(len(obsticles)))
    obsts_living = np.zeros( len(obsticles) )
    for c, r  in found_obsticles:
        closest = None
        minDist = None
        for i in range(len(obsticles)):
            if (minDist == None and obsticles[i].dist(c[0], c[1], time) < ( 0.90 )) or ( minDist != None and obsticles[i].dist( c[0], c[1], time) < minDist):
                minDist = obsticles[i].dist( c[0], c[1], time)
                closest = i
            # else:
            #     print(obsticles[i].dist(c[0], c[1], time))
        if closest == None:
            obsticles.append( Obsticle(c[0], c[1], time, r) )
        else:
            if time > obsticles[closest].lasttime:
                obsticles[closest].update( obsticles[closest].nextState( c[0], c[1], time ), time, r )
                obsts_living[closest] = 1
    for i, h in zip( range(len(obsts_living)), obsts_living ):
        if h == 0:
            obsticles[i].age()

    obsticles = [ o for o in obsticles if o.is_healthy() ]
    # print("Processed Obsticals", str(ttime.time() - tStart))
    #print( '\n'.join( map( str, obsticles ) ) )


def handle_obsticle_request(req):
    global obsticles
    obs = Obstacles()
    obs.count = len(obsticles)
    for o in obsticles:
        obs.obstacles.append( o.generate_obsticle_prediction(req.dt, req.steps) )
    return obs





def init_node():
    global world_map, transformer
    global map_resolution, cloud_pub, path_pub, sphere_pub

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
        path_pub = rospy.Publisher("obst_paths", MarkerArray, queue_size=5)
        sphere_pub = rospy.Publisher("obst_sizes", MarkerArray, queue_size=5)
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
