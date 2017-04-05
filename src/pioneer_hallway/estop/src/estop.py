#!/usr/bin/env python

import rospy
import roslib
import math
from tf import TransformListener, transformations
import subprocess
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point32
from geometry_msgs.msg import Quaternion, PolygonStamped, Twist
from std_srvs.srv import Empty

#global variables
rosAriaPose = Odometry()
amclPose = PoseWithCovarianceStamped()
disablePublisher = None
#testCloudPub = None
#currentFramePub = None
predictedFramePub = None
cmdVelPub = None

#used in rotations
frame = 'map'

#values in meters
robotLength = 0.511 #value grabbed from /usr/local/Aria/params/p3dx.p
robotWidth = 0.425 #value grabbed from /usr/local/Aria/params/p3dx.p
robotLCushion = 0.16
robotWCushion = 0.12
robotLengthFromCenter = (robotLength + robotLCushion) / 2
robotWidthFromCenter = (robotWidth + robotWCushion) / 2
sonarOffset = -0.198 #value grabbed from pioneer3dx.xacro
laserOffset = 0.17 #value grabbed from pioneer3dx.xacro

#value in m/s**2
maxDecel = 0.6
ariaFactor = 0.7

#calculates the distance between a point and a line
#lineEnd and lineStart define the end and start points of the line
#point is the point we want to calculate the distance to
def calculateDistance(lineEnd, lineStart, point):
	distance = abs((lineEnd.y - lineStart.y) * point.x - (lineEnd.x - lineStart.x) * point.y + lineEnd.x * lineStart.y - lineEnd.y * lineStart.x) / pow(pow(lineEnd.y - lineStart.y,2) + pow(lineEnd.x - lineStart.x,2), 1/2)
	return distance

#calculates the distance between two points
def calculateEuclideanDistance(p1, p2):
	return sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def polar_to_euclidean( angles, ranges ):
    return [ [ np.cos(theta)*r, np.sin(theta)*r ] for theta, r in zip( angles, ranges ) ]

#rotates a point given a quaternion
#pin is the point that will be rotated
#q is the quaternion that will be used to ratet the point
def rotatePose(pin, q):
	x = q.w * q.w * pin.x - 2 * q.z * q.w * pin.y + q.x * q.x * pin.x + 2 * q.y *q.x * pin.y - q.z * q.z * pin.x - q.y * q.y * pin.x;
	y = 2 * q.x * q.y * pin.x + q.y * q.y * pin.y + 2 * q.w * q.z * pin.x - q.z *q.z * pin.y + q.w * q.w * pin.y - q.x * q.x * pin.y;
	pin.x = x
	pin.y = y

#receives and stores the current velocity of the robot
def rosAriaPoseCallback(odom):
	global rosAriaPose
	rosAriaPose = odom
	
#receives and stores the current position of the robot
def amclPoseCallback(pose):
	global amclPose
	amclPose = pose

def calculatePolygons(deltaT):
	#rospy.loginfo("%f", deltaT)
	#get current pose
	currPose = amclPose.pose.pose
	currTwist = rosAriaPose.twist.twist
	
	#create polygons
	polyCurr = PolygonStamped()
	polyPred = PolygonStamped()
	
	#rotate robot frame
	robotFrameA = Point32(robotLengthFromCenter, robotWidthFromCenter, 0)
	robotFrameB = Point32(robotLengthFromCenter, -robotWidthFromCenter, 0)
	robotFrameC = Point32(-robotLengthFromCenter, robotWidthFromCenter, 0)
	robotFrameD = Point32(-robotLengthFromCenter, -robotWidthFromCenter, 0)
	rotatePose(robotFrameA, currPose.orientation)
	rotatePose(robotFrameB, currPose.orientation)
	rotatePose(robotFrameC, currPose.orientation)
	rotatePose(robotFrameD, currPose.orientation)
	
	#calculate current vertices
	cp0 = Point32(currPose.position.x + robotFrameA.x, currPose.position.y + robotFrameA.y, 0)
	cp1 = Point32(currPose.position.x + robotFrameB.x, currPose.position.y + robotFrameB.y, 0)
	cp2 = Point32(currPose.position.x + robotFrameC.x, currPose.position.y + robotFrameC.y, 0)
	cp3 = Point32(currPose.position.x + robotFrameD.x, currPose.position.y + robotFrameD.y, 0)
	
	#predict vertices
	#calculates linear movement
	delta = Point32(currTwist.linear.x * deltaT, currTwist.linear.y * deltaT, 0)
	rotatePose(delta, currPose.orientation)
	#calculates angular movement
	newOrientation = Quaternion()
	newOrientation.z = math.sin(currTwist.angular.z * deltaT / 2)
	newOrientation.w = math.cos(currTwist.angular.z * deltaT / 2)
	rotatePose(robotFrameA, newOrientation)
	rotatePose(robotFrameB, newOrientation)
	rotatePose(robotFrameC, newOrientation)
	rotatePose(robotFrameD, newOrientation)
	pp0 = Point32(currPose.position.x + robotFrameA.x + delta.x, currPose.position.y + robotFrameA.y + delta.y, 0)
	pp1 = Point32(currPose.position.x + robotFrameB.x + delta.x, currPose.position.y + robotFrameB.y + delta.y, 0)
	pp2 = Point32(currPose.position.x + robotFrameC.x + delta.x, currPose.position.y + robotFrameC.y + delta.y, 0)
	pp3 = Point32(currPose.position.x + robotFrameD.x + delta.x, currPose.position.y + robotFrameD.y + delta.y, 0)
	
	#create final polygons
	polyCurr.polygon.points = [None] * 4
	polyCurr.polygon.points[0] = cp0
	polyCurr.polygon.points[1] = cp1
	polyCurr.polygon.points[2] = cp3
	polyCurr.polygon.points[3] = cp2
	polyCurr.header.frame_id = frame
	
	polyPred.polygon.points = [None] * 4
	polyPred.polygon.points[0] = pp0
	polyPred.polygon.points[1] = pp1
	polyPred.polygon.points[2] = pp3
	polyPred.polygon.points[3] = pp2
	polyPred.header.frame_id = frame
	
	#publish polygons
	#currentFramePub.publish(polyCurr)
	predictedFramePub.publish(polyPred)

	return polyPred

def triggerEstop():
	disablePublisher()
	rospy.loginfo('Possible collision detected.')
	subprocess.call(['/usr/bin/canberra-gtk-play','--id','suspend-error'])
	#possible sound files are in /usr/share/sounds/freedesktop/stereo
	exit()

#uses sonar readings to predict collisions
def sonarCallback(data):
	pose = amclPose.pose.pose
	pc = PointCloud()
	pc.header.frame_id = "map"
	pc.points = [0] * len(data.points)
	counter = 0
	for point in data.points:
		tfq = transformations.quaternion_from_euler(0, 0, 0)
		q = Quaternion(tfq[0], tfq[1], tfq[2], tfq[3])
		p = Point32(point.x, point.y, 0)
		rotatePose(p, q)
		#p.x = p.x
		rotatePose(p, pose.orientation)
		p.x = pose.position.x + p.x
		p.y = pose.position.y + p.y
		pc.points[counter] = p
		counter = counter + 1
	#testCloudPub.publish(pc)
	evaluateReadings(pc)

#uses laser readings to predict collisions
def laserCallback(data):
	pose = amclPose.pose.pose
	points = polar_to_euclidean( [ data.angle_min + (i * data.angle_increment)  for i in range(len(data.ranges))], data.ranges)
	pc = PointCloud()
	pc.header.frame_id = "map"
	pc.points = [0] * len(points)		
	counter = 0
	for point in points:
		p = Point32(laserOffset + point[0], point[1], 0)
		rotatePose(p, pose.orientation)
		p.x = pose.position.x + p.x
		p.y = pose.position.y + p.y
		pc.points[counter] = p
		counter = counter + 1			
	evaluateReadings(pc)
	#testCloudPub.publish(pc)

#uses laser readings to simulate the sonar and predict collisions
def sonarLaserCallback(data):
	pose = amclPose.pose.pose
	points = polar_to_euclidean( [ data.angle_min + (i * data.angle_increment)  for i in range(len(data.ranges))], data.ranges)
	pc = PointCloud()
	pc.header.frame_id = "map"
	pc.points = [0] * len(points)
	counter = 0
	for point in points:
		tfq = transformations.quaternion_from_euler(0, 0, 3.14)
		q = Quaternion(tfq[0], tfq[1], tfq[2], tfq[3])
		p = Point32(point[0], point[1], 0)
		rotatePose(p, q)
		p.x = sonarOffset + p.x
		rotatePose(p, pose.orientation)
		p.x = pose.position.x + p.x
		p.y = pose.position.y + p.y
		pc.points[counter] = p
		counter = counter + 1
	evaluateReadings(pc)
	#testCloudPub.publish(pc)

def evaluateReadings(pc):
	#calculate deltaT
	currSpeed = rosAriaPose.twist.twist
	if currSpeed.linear.x <= -0.01 or currSpeed.linear.x >= 0.01:
		deltaT = max(abs((currSpeed.linear.x / ariaFactor) / maxDecel), 0.5)
		timeIncrement = deltaT / 4
		currTime = timeIncrement
		while currTime <= deltaT:
			polygon = calculatePolygons(currTime)
			#try:
			#	predictedFramePub.publish(polygon)
			#except:
			#	pass
			minX = 10000
			minY = 10000
			maxX = -10000
			maxY = -10000
			for point in polygon.polygon.points:
				if point.x < minX:
					minX = point.x
				if point.x > maxX:
					maxX = point.x
				if point.y < minY:
					minY = point.y
				if point.y > maxY:
					maxY = point.y
			for point in pc.points:
				if (point.x > maxX or point.x < minX or 
				  point.y > maxY or point.y < minY):
					continue
				else:
					if ((point.x >= minX and point.x <= maxX) and
					  (point.y >= minY and point.y <= maxY)):
						triggerEstop()
			currTime = currTime + timeIncrement

def estop():
	global disablePublisher
	global predictedFramePub
	#global currentFramePub
	#global testCloudPub
	global cmdVelPub	
	global transformer
	global ariaFactor
	
	#initialize node
	rospy.init_node('pioneer_estop', anonymous=False)
	
	#initialize frame publishers
	#currentFramePub = rospy.Publisher('estop_current_frame', PolygonStamped, queue_size=1)
	predictedFramePub = rospy.Publisher('estop_predicted_frame', PolygonStamped, queue_size=1)
	#testCloudPub = rospy.Publisher('testCloud', PointCloud, queue_size=1)
	
	#load configuration parameters
	laserTopic = rospy.get_param("laserTopic", "RosAria/lms5XX_1_laserscan")
	ariaFactor = float(rospy.get_param("ariaFactor", "0.7"))
	sim = rospy.get_param("simulation", "False")
	
	rospy.loginfo("%f", ariaFactor)
	
	#connect to disable_cmd_vel_publisher service
	rospy.wait_for_service('disable_cmd_vel_publisher')
	
	#subscribe to laser, sonar, amcl and odometry
	transformer = TransformListener()
	rospy.Subscriber(laserTopic, LaserScan, laserCallback)
	rospy.Subscriber('RosAria/pose', Odometry, rosAriaPoseCallback)
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amclPoseCallback)
	if sim == True:
		rospy.Subscriber('sonar_b', LaserScan, sonarLaserCallback)
		rospy.loginfo('simulation')
	else:
		rospy.Subscriber('RosAria/sonar', PointCloud, sonarCallback)
		
	#initialize connection to disable_cmd_vel_publisher service
	disablePublisher = rospy.ServiceProxy('disable_cmd_vel_publisher', Empty)
	
	rospy.loginfo('estop ready')
	rospy.spin()
		

if __name__ == '__main__':
	estop()
