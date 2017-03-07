#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"

ros::Publisher pub;

void poseCallback(const nav_msgs::Odometry& msg)
{
  geometry_msgs::Point pp = msg.pose.pose.position ;
  geometry_msgs::Quaternion qq = msg.pose.pose.orientation ;
  tf::Quaternion tf_quat(qq.x, qq.y, qq.z, qq.w) ;
  
  geometry_msgs::Twist current_Twist = msg.twist.twist ;
  
  geometry_msgs::TransformStamped odom_trans;
  
  odom_trans.transform.translation.x = pp.x ;
  odom_trans.transform.translation.y = pp.y ;
  odom_trans.transform.translation.z = pp.z ;
  
  geometry_msgs::Quaternion geo_Quat ;
  tf::quaternionTFToMsg(tf_quat, geo_Quat) ;
  odom_trans.transform.rotation = geo_Quat ;

  odom_trans.header.stamp = ros::Time::now() ;
  odom_trans.header.frame_id = "odom" ;
  odom_trans.child_frame_id = "base_link" ;

  nav_msgs::Odometry odom ;
  odom.header.stamp = odom_trans.header.stamp ;
  odom.header.frame_id = "odom" ;

  //set the position
  odom.pose.pose.position.x = odom_trans.transform.translation.x ;
  odom.pose.pose.position.y = odom_trans.transform.translation.y ;
  odom.pose.pose.position.z = odom_trans.transform.translation.z ;
  odom.pose.pose.orientation = geo_Quat ;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = current_Twist.linear.x ;
  odom.twist.twist.linear.y = current_Twist.linear.y ;
  odom.twist.twist.linear.z = current_Twist.linear.z ;

  odom.twist.twist.angular.x= current_Twist.angular.x ;
  odom.twist.twist.angular.y= current_Twist.angular.y ;
  odom.twist.twist.angular.z= current_Twist.angular.z ;

  //publish the message
  pub.publish(odom);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer3dx_odometry_publisher") ;

  ros::NodeHandle nHandle1 ;
  ros::NodeHandle nHandle2 ; 
  
  pub = nHandle2.advertise<nav_msgs::Odometry>("odom", 50) ;
  ros::Subscriber sub = nHandle1.subscribe("/RosAria/pose", 10, &poseCallback) ;
  
  ros::spin() ;
  return 0 ;
}


