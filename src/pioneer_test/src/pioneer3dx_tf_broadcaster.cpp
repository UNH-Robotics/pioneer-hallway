#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

void poseCallback(const nav_msgs::Odometry& msg)
{
  static tf::TransformBroadcaster br ;
	
  tf::Transform transform ;
  geometry_msgs::Point pp = msg.pose.pose.position ;
  geometry_msgs::Quaternion qq = msg.pose.pose.orientation ;
  transform.setOrigin( tf::Vector3(pp.x, pp.y, pp.z) ) ;
  tf::Quaternion q(qq.x, qq.y, qq.z, qq.w) ;
  transform.setRotation(q) ;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link")) ;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer3dx_tf_broadcaster") ;
  
  ros::NodeHandle nHandle ;
  
  ros::Subscriber sub = nHandle.subscribe("/RosAria/pose", 10, &poseCallback) ;
  
  ros::spin() ;
  
  return 0 ;
}
