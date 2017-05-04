#!/usr/bin/env python

import rospy
import roslib
import math
import subprocess
import numpy as np
from tf import TransformListener, transformations
from sensor_msgs.msg import LaserScan, PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point32
from geometry_msgs.msg import Quaternion, PolygonStamped, Twist
from std_srvs.srv import Empty

#global variables
rosAriaPose = Odometry()
amclPose = PoseWithCovarianceStamped()
disablePublisher = None
collisionFramePub = None
predictedFramePub = None
cmdVelPub = None
collision = False
#testCloudPub = None

#used in rotations
frame = 'map'

#values in meters
robotLength = 0.511 #value grabbed from /usr/local/Aria/params/p3dx.p
robotWidth = 0.381 #value grabbed from /usr/local/Aria/params/p3dx.p
robotLCushion = 0.08
robotWCushion = 0.08
robotLengthFromCenter = (robotLength + robotLCushion) / 2
robotWidthFromCenter = (robotWidth + robotWCushion) / 2
sonarOffset = -0.198 #value grabbed from pioneer3dx.xacro
laserOffset = 0.17 #value grabbed from pioneer3dx.xacro

#value in m/s**2
maxDecel = 0.6

def polar_to_euclidean( angles, ranges ):
    return [ [ np.cos(theta)*r, np.sin(theta)*r ] for 
           theta, r in zip( angles, ranges ) ]

#rotates a point given a quaternion
#pin is the point that will be rotated
#q is the quaternion that will be used to ratet the point
def rotatePose(pin, q):
	x = (q.w * q.w * pin.x - 2 * q.z * q.w * pin.y + q.x * q.x * pin.x + 
	    2 * q.y *q.x * pin.y - q.z * q.z * pin.x - q.y * q.y * pin.x)
	y = (2 * q.x * q.y * pin.x + q.y * q.y * pin.y + 2 * q.w * q.z * pin.x - 
	    q.z *q.z * pin.y + q.w * q.w * pin.y - q.x * q.x * pin.y)
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

#calculates de position of the robot at a given time based on the current speed
def calculatePolygons(deltaT):
	#get current pose and speed
	currPose = amclPose.pose.pose
	currTwist = rosAriaPose.twist.twist
	
	#create polygons
	#polyCurr = PolygonStamped()
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
	pp0 = Point32(currPose.position.x + robotFrameA.x + delta.x, 
	       currPose.position.y + robotFrameA.y + delta.y, 0)
	pp1 = Point32(currPose.position.x + robotFrameB.x + delta.x, 
	       currPose.position.y + robotFrameB.y + delta.y, 0)
	pp2 = Point32(currPose.position.x + robotFrameC.x + delta.x, 
	       currPose.position.y + robotFrameC.y + delta.y, 0)
	pp3 = Point32(currPose.position.x + robotFrameD.x + delta.x, 
	       currPose.position.y + robotFrameD.y + delta.y, 0)
	
	#creates polygon
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

def triggerEstop(polygon):
	global collision
	collision = True
	disablePublisher()
	collisionFramePub.publish(polygon)
	rospy.loginfo('Possible collision detected.')
	subprocess.call(['/usr/bin/canberra-gtk-play','--id','suspend-error'])
	#possible sound files are in /usr/share/sounds/freedesktop/stereo
	exit()

#uses sonar readings to predict collisions
def sonarCallback(data):
	global collision
	currSpeed = rosAriaPose.twist.twist
	if not collision and (currSpeed.linear.x >= abs(0.02)):
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
		if not collision:
			evaluateReadings(pc)

#uses laser readings to predict collisions
def laserCallback(data):
	global collision
	currSpeed = rosAriaPose.twist.twist
	if not collision and (currSpeed.linear.x >= abs(0.02)):
		pose = amclPose.pose.pose
		points = polar_to_euclidean( [ data.angle_min + (i * data.angle_increment)  
							for i in range(len(data.ranges))], data.ranges)
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
		if not collision:
			evaluateReadings(pc)
		#testCloudPub.publish(pc)

#uses laser readings to simulate the sonar and predict collisions
def sonarLaserCallback(data):
	global collision
	currSpeed = rosAriaPose.twist.twist
	if not collision and (currSpeed.linear.x >= abs(0.02)):
		pose = amclPose.pose.pose
		points = polar_to_euclidean( [ data.angle_min + (i * data.angle_increment)  
							for i in range(len(data.ranges))], data.ranges)
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
		if not collision:
			evaluateReadings(pc)
		#testCloudPub.publish(pc)

#evaluates a give point cloud to check for collisions
def evaluateReadings(pc):
	#calculate deltaT
	global collision
	currSpeed = rosAriaPose.twist.twist
	
	#check for collisions only if the robot is moving
	if currSpeed.linear.x <= -0.02 or currSpeed.linear.x >= 0.02:
		deltaT = abs((currSpeed.linear.x) / maxDecel)
		timeIncrement = deltaT / 4
		currTime = timeIncrement
		while currTime <= deltaT and not collision:
			polygon = calculatePolygons(currTime)
			A = Point32(polygon.polygon.points[1].x - polygon.polygon.points[0].x, 
			     polygon.polygon.points[1].y - polygon.polygon.points[0].y, 0)
			B = Point32(polygon.polygon.points[3].x - polygon.polygon.points[0].x, 
			     polygon.polygon.points[3].y - polygon.polygon.points[0].y, 0)			
			Asquared = pow(A.x,2) + pow(A.y,2)
			Bsquared = pow(B.x,2) + pow(B.y,2)
			for point in pc.points:
				p = Point32(point.x - polygon.polygon.points[0].x, 
				     point.y - polygon.polygon.points[0].y, 0)
				if (0 <= p.x * A.x + p.y * A.y <= Asquared):
					if (0 <= p.x * B.x + p.y * B.y <= Bsquared):	
						triggerEstop(polygon)
			currTime = currTime + timeIncrement
		predictedFramePub.publish(polygon)

def estop():
	global disablePublisher
	global predictedFramePub
	global collisionFramePub
	global cmdVelPub	
	#global testCloudPub
	
	#initialize node
	rospy.init_node('pioneer_estop', anonymous=False)
	
	#initialize frame publishers
	collisionFramePub = rospy.Publisher('estop_collision_frame', PolygonStamped, queue_size=1)
	predictedFramePub = rospy.Publisher('estop_predicted_frame', PolygonStamped, queue_size=1)
	#testCloudPub = rospy.Publisher('testCloud', PointCloud, queue_size=1)
	
	#load configuration parameters
	laserTopic = rospy.get_param("laserTopic", "RosAria/lms5XX_1_laserscan")
	sim = rospy.get_param("simulation", "False")
	
	#connect to disable_cmd_vel_publisher service
	rospy.wait_for_service('disable_cmd_vel_publisher')
	
	#subscribe to laser, sonar, amcl and odometry
	rospy.Subscriber(laserTopic, LaserScan, laserCallback)
	rospy.Subscriber('RosAria/pose', Odometry, rosAriaPoseCallback)
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amclPoseCallback)
	if sim == True:
		rospy.Subscriber('sonar_b', LaserScan, sonarLaserCallback)
	else:
		rospy.Subscriber('RosAria/sonar', PointCloud, sonarCallback)
		
	#initialize connection to disable_cmd_vel_publisher service
	disablePublisher = rospy.ServiceProxy('disable_cmd_vel_publisher', Empty)
	
	rospy.loginfo('estop ready')
	rospy.spin()
		

if __name__ == '__main__':
	estop()
