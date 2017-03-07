#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "pioneer_odom_publisher") ;
  ros::NodeHandle n1 ;
  ros::NodeHandle n2 ;
  ros::Rate loop_rate(10) ;
  ros::ServiceClient client ;
      
  ros::Publisher pub = n1.advertise<nav_msgs::Odometry>("odom", 50) ;
  
  std::string modelName = (std::string)"pioneer" ;
  std::string relativeEntityName = (std::string)"world" ;

  gazebo_msgs::GetModelState getModelState ;
  geometry_msgs::Point pp ;
  geometry_msgs::Quaternion qq ;
  geometry_msgs::Twist current_Twist ;

  while (ros::ok())
  {
      client = n2.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
      getModelState.request.model_name = modelName ;
      getModelState.request.relative_entity_name = relativeEntityName ;
      client.call(getModelState) ;

      pp = getModelState.response.pose.position ;
      qq = getModelState.response.pose.orientation ;
      tf::Quaternion tf_quat(qq.x, qq.y, qq.z, qq.w) ;
      
      current_Twist = getModelState.response.twist ;
      
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

      ros::spinOnce();
      loop_rate.sleep() ;
 }
 
 return 0 ;
 
}


