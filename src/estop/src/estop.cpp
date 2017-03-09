#include <vector>
#include <boost/thread/mutex.hpp>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Polygon.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "footprint_publisher/Footprint.h"

double maxLinearDecceleration = 0.2; //TODO CHANGE TO CORRECT VALUE
double maxAngularDecceleration = 0.2; //TODO CHANGE TO CORRECT VALUE
double deltaT = 0.1;
double heading = 0;

laser_geometry::LaserProjection laserProjector;
ros::ServiceClient motorDisabler;
geometry_msgs::Polygon footprint;
geometry_msgs::Twist velocity;
boost::mutex lock;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/* > 0 p2 is left
 * = 0 p2 is on line
 * < 0 p2 is right
 */
int isLeft(const geometry_msgs::Point32& p0,
	    const geometry_msgs::Point32& p1,
	    const geometry_msgs::Point32& p2) {
  return ((p1.x - p0.x) * (p2.y - p0.y) -
	  (p2.x - p0.x) * (p1.y - p0.y));
}

bool pointInConvexPoly(const geometry_msgs::Point32& pt, const geometry_msgs::Polygon& footprint) {
  auto points = footprint.points;
  int wn = 0;
  for(unsigned int i = 0; i < points.size(); i++) {	
    auto p0 = points[i % points.size()];
    auto p1 = points[(i + 1) % points.size()];
    if(p0.y <= pt.y) {
      if(p1.y > pt.y) {
	    if(isLeft(p0, p1, pt) > 0) {
	      ++wn;
	    }
      }
    } else {
	  if(p1.y <= pt.y) {
	    if(isLeft(p0, p1, pt) < 0) {
	      --wn;
	    }
	  }
    }
  }
  return wn != 0;
}

void advance(geometry_msgs::Polygon &footprint,
	     geometry_msgs::Twist &velocity,
	     double maxLinearDecceleration,
	     double maxAngularDecceleration,
	     double deltaT) {
  //translate
  if(sgn(velocity.linear.x) > 0) { 
	  velocity.linear.x -= maxLinearDecceleration * deltaT; 
  }
  else { 
	  velocity.linear.x += maxLinearDecceleration * deltaT; 
  }

  double magnitude = velocity.linear.x * deltaT;

  for(auto iter = footprint.points.begin(); iter != footprint.points.end(); ++iter) {
    auto pt = *iter;
    pt.x -= magnitude * cos(heading);
    pt.y -= magnitude * sin(heading);
  }

  //rotate
  if(sgn(velocity.angular.z) > 0) { 
    velocity.angular.z -= maxAngularDecceleration * deltaT; 
  }
  else { 
	velocity.angular.z += maxAngularDecceleration * deltaT; 
  }
}

void setVelocity(const geometry_msgs::Twist &twist) {
  lock.lock();
  velocity = twist;
  lock.unlock();
}

const geometry_msgs::Twist getVelocity() {
  geometry_msgs::Twist vel;
  lock.lock();
  vel = velocity;
  lock.unlock();
  return vel;
}

void odomListener(const nav_msgs::Odometry::ConstPtr& msg) {
  auto twist = msg->twist.twist;
  setVelocity(twist);
}

/*sensor_msgs::LaserScan sonarToLaserScan(const p2os_msgs::SonarArray& msg) {
  sensor_msgs::LaserScan sonar;

  sonar.header = msg.header;
  sonar.time_increment = 0;
  sonar.scan_time = 0;
  sonar.range_min = 0;
  sonar.range_max = 0;
  sonar.angle_min = 0; //update
  sonar.angle_max = 0; //update
  sonar.angle_increment = 0; //update

  for(auto iter = msg.ranges.begin(); iter != msg.ranges.end(); ++iter) {
    sonar.ranges.push_back(*iter);
  }

  return sonar;
}*/

void checkLaserScanForCollisions(const sensor_msgs::LaserScan &scan) {
  geometry_msgs::Twist vel = getVelocity();
  geometry_msgs::Polygon predicted = footprint;
  double startLinearVel = vel.linear.x;
  
  sensor_msgs::PointCloud cloud;
  laserProjector.projectLaser(scan, cloud);
  while(sgn(startLinearVel) == sgn(vel.linear.x)) {
    advance(predicted, vel, maxLinearDecceleration, maxAngularDecceleration, deltaT);
    for(auto iter = cloud.points.begin(); iter != cloud.points.end(); ++iter) {
      if(pointInConvexPoly(*iter, predicted)) {
		std_srvs::Empty srv;
		motorDisabler.call(srv);
		ROS_ERROR("POSSIBLE COLLISION DETECTED! TURNING OFF MOTORS!");
		exit(0);
      }
    }
  }
  ROS_INFO("Finished simulation");
}

/*void sonarListener(const p2os_msgs::SonarArray::ConstPtr& msg) {
  sensor_msgs::LaserScan scan = sonarToLaserScan(*msg);
  checkLaserScanForCollisions(scan);
}*/

void laserListener(const sensor_msgs::LaserScan::ConstPtr& msg) {
  checkLaserScanForCollisions(*msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "estop_node");
  ros::NodeHandle n;

  ros::Subscriber odomSubscriber = n.subscribe("odom", 1, odomListener);
//  ros::Subscriber sonarSubscriber = n.subscribe("RosAria/sonar", 1, sonarListener);
  ros::Subscriber laserSubcriber = n.subscribe("base_scan", 1, laserListener);

  motorDisabler = n.serviceClient<std_srvs::Empty>("disable_motors");

  footprint_publisher::Footprint::Request request;
  footprint_publisher::Footprint::Response response;
  if(ros::service::call("footprint", request, response)) {
    for(auto iter = response.footprint.points.begin(); iter != response.footprint.points.end(); ++iter) {
      footprint.points.push_back(*iter);
    }
  } else {
    ROS_ERROR("Failed to call service footprint");
    return 1;
  }

  ros::spin();

  return 0;
}
