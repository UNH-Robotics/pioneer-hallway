#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"

class OdomToMap{
  public:
    OdomToMap(ros::NodeHandle) ;
    ~OdomToMap() {}
  private:
    ros::Subscriber subOdom ;
    ros::Publisher pubOdomMap ;
    void odomCallback(const nav_msgs::Odometry&) ;

    tf::TransformListener listener ;
    tf::StampedTransform transform ;
    nav_msgs::Odometry odom_map ;
} ;

OdomToMap::OdomToMap(ros::NodeHandle nh){
  subOdom = nh.subscribe("odom", 10, &OdomToMap::odomCallback, this) ;
  pubOdomMap = nh.advertise<nav_msgs::Odometry>("odom_map", 10) ;
  try{
      listener.lookupTransform("/map", "odom", ros::Time(0), transform) ;
    }
  catch (tf::TransformException &ex){
    ROS_ERROR("%s", ex.what()) ;
    ros::Duration(1.0).sleep() ;
  }
}

void OdomToMap::odomCallback(const nav_msgs::Odometry& msg)
{
  ros::Rate rate(10.0) ;
  while (ros::ok()){
    try{
      listener.lookupTransform("/map", "odom", ros::Time(0), transform) ;
    }
    catch (tf::TransformException &ex){
      ROS_ERROR("%s", ex.what()) ;
      ros::Duration(1.0).sleep() ;
      continue ;
    }
  
    nav_msgs::Odometry odom_map = msg ;
    odom_map.header.stamp = ros::Time::now() ;
    odom_map.header.frame_id = "/map" ;
    odom_map.child_frame_id = "odom" ;
    odom_map.pose.pose.position.x = msg.pose.pose.position.x + transform.getOrigin().x() ;
    odom_map.pose.pose.position.y = msg.pose.pose.position.y + transform.getOrigin().y() ;
    odom_map.pose.pose.position.z = msg.pose.pose.position.z + transform.getOrigin().z() ;
    odom_map.pose.pose.orientation.x = msg.pose.pose.orientation.x + transform.getRotation().x() ;
    odom_map.pose.pose.orientation.y = msg.pose.pose.orientation.y + transform.getRotation().y() ;
    odom_map.pose.pose.orientation.z = msg.pose.pose.orientation.z + transform.getRotation().z() ;
    odom_map.pose.pose.orientation.w = msg.pose.pose.orientation.w + transform.getRotation().w() ;
    
/*    ROS_INFO_STREAM("odom: [" << msg.pose.pose.position.x << "," << msg.pose.pose.position.y << "," << msg.pose.pose.position.z << "]" ) ;
    ROS_INFO_STREAM("tf_o: [" << transform.getOrigin().x() << "," << transform.getOrigin().y() << "," << transform.getOrigin().z() << "]") ;
    ROS_INFO_STREAM("2map: [" << odom_map.pose.pose.position.x << "," << odom_map.pose.pose.position.y << "," << odom_map.pose.pose.position.z << "]" ) ;*/
    //publish the message
    pubOdomMap.publish(odom_map);
    ros::spinOnce();
  }
  
}
