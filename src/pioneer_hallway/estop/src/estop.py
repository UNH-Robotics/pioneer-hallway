#!/usr/bin/env python

import rospy
import math
import subprocess
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point32
from geometry_msgs.msg import Twist, Quaternion, PolygonStamped
from std_srvs.srv import Empty

#global variables
rosAriaPose = Odometry()
amclPose = PoseWithCovarianceStamped()
disablePublisher = None
currentFramePub = None
predictedFramePub = None
cmdVelPub = None

#value in seconds
deltaT = 4.5

#values in meters
robotLength = 0.455
robotWidth = 0.381
robotLCushion = 0.16
robotWCushion = 0.12
frame = 'map'
robotLengthFromCenter = (robotLength + robotLCushion) / 2
robotWidthFromCenter = (robotWidth + robotWCushion) / 2

#calculates the distance between a point and a line
#lineEnd and lineStart define the end and start points of the line
#point is the point we want to calculate the distance to
def calculateDistance(lineEnd, lineStart, point):
	distance = abs((lineEnd.y - lineStart.y) * point.x - (lineEnd.x - lineStart.x) * point.y + lineEnd.x * lineStart.y - lineEnd.y * lineStart.x) / pow(pow(lineEnd.y - lineStart.y,2) + pow(lineEnd.x - lineStart.x,2), 1/2)
	return distance

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

#predict collisions and stops the motors if necessary
def laserCallback(sensor_data):
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
	
	#check for collisions
	laserDistance = sensor_data.ranges[len(sensor_data.ranges)/2]
	distFront = calculateDistance(pp0, pp1, currPose.position)
	distRight = calculateDistance(pp1, pp3, currPose.position)
	distLeft = calculateDistance(pp3, pp2, currPose.position)
	distBack = calculateDistance(pp2, pp0, currPose.position)
	for point in sensor_data.ranges:
		if (laserDistance <= distFront or
		    laserDistance <= distLeft or 
		    laserDistance <= distRight or
		    laserDistance <= distBack):
			disablePublisher()
			rospy.loginfo('Possible collision detected.')
			subprocess.call(['/usr/bin/canberra-gtk-play','--id','suspend-error'])
			exit()
	
	#publish polygons
	currentFramePub.publish(polyCurr)
	predictedFramePub.publish(polyPred)


def estop():
	global disablePublisher
	global predictedFramePub
	global currentFramePub
	global cmdVelPub	
	rospy.init_node('pioneer_estop', anonymous=False)
	laserTopic = rospy.get_param("laserTopic", "lms5XX_1_laserscan")
	rospy.wait_for_service('disable_cmd_vel_publisher')
	rospy.Subscriber(laserTopic, LaserScan, laserCallback)
	rospy.Subscriber('RosAria/pose', Odometry, rosAriaPoseCallback)
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amclPoseCallback)
	currentFramePub = rospy.Publisher('estop_current_frame', PolygonStamped, queue_size=1)
	predictedFramePub = rospy.Publisher('estop_predicted_frame', PolygonStamped, queue_size=1)
	disablePublisher = rospy.ServiceProxy('disable_cmd_vel_publisher', Empty)
	rospy.loginfo('estop ready')
	rospy.spin()
		

if __name__ == '__main__':
	estop()
