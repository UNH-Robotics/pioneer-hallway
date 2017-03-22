#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from std_srvs.srv import Empty

pose = None
currentFramePub = None
predictedFramePub = None
cmdVelPub = None

#value in seconds
deltaT = 1

#values in meters
robotLength = 0.455
robotWidth = 0.381
robotLCushion = 0.16
robotWCushion = 0.12
frame = 'odom'
robotLengthFromCenter = (robotLength + robotLCushion) / 2
robotWidthFromCenter = (robotWidth + robotWCushion) / 2

def calculateDistance(lineEnd, lineStart, point):
	distance = abs((lineEnd.y - lineStart.y) * point.x - (lineEnd.x - lineStart.x) * point.y + lineEnd.x * lineStart.y - lineEnd.y * lineStart.x) / pow(pow(lineEnd.y - lineStart.y,2) + pow(lineEnd.x - lineStart.x,2), 1/2)
	return distance

def rotatePose(pin, q):
	x = q.w * q.w * pin.x - 2 * q.z * q.w * pin.y + q.x * q.x * pin.x + 2 * q.y *q.x * pin.y - q.z * q.z * pin.x - q.y * q.y * pin.x;
	y = 2 * q.x * q.y * pin.x + q.y * q.y * pin.y + 2 * q.w * q.z * pin.x - q.z *q.z * pin.y + q.w * q.w * pin.y - q.x * q.x * pin.y;
	pin.x = x
	pin.y = y

def poseCallback(odom):
	global pose
	pose = odom

def laserCallback(sensor_data):
	#get current pose
	currPose = pose.pose.pose
	currTwist = pose.twist.twist
	
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
	delta = Point32(currTwist.linear.x * deltaT, currTwist.linear.y * deltaT, 0)
	rotatePose(delta, currPose.orientation)
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
	currentFramePub.publish(polyCurr)
	predictedFramePub.publish(polyPred)
	
	laserDistance = sensor_data.ranges[len(sensor_data.ranges)/2]
	predictedDistance = calculateDistance(pp0, pp1, currPose.position)
	if laserDistance <= predictedDistance:
		rospy.loginfo('Possible collision detected. %f  -  %f', laserDistance, predictedDistance)
		#cmdVelPub.publish(Twist())
		disableMotors = rospy.ServiceProxy('disable_motors', Empty)
		disableMotors()

def estop():
	global predictedFramePub
	global currentFramePub
	global cmdVelPub
	rospy.wait_for_service('disable_motors')
	rospy.loginfo('Connected to motors')
	rospy.init_node('pioneer_estop', anonymous=False)
	rospy.Subscriber('base_scan', LaserScan, laserCallback)
	rospy.Subscriber('odom', Odometry, poseCallback)
	currentFramePub = rospy.Publisher('estop_current_frame', PolygonStamped, queue_size=1)
	predictedFramePub = rospy.Publisher('estop_predicted_frame', PolygonStamped, queue_size=1)
	cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	rospy.spin()
		

if __name__ == '__main__':
	estop()
