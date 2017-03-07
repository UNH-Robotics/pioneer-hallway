#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

ros::Publisher pub;

void sonarPC2Callback(const sensor_msgs::LaserScan& msg)
{
  sensor_msgs::LaserScan newscan = msg ;
  newscan.header.stamp = ros::Time::now() ;
  newscan.header.frame_id = "laser" ;
  
  pub.publish(newscan) ;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rplidar_laser_publisher");

  ros::NodeHandle nHandpub;
  pub = nHandpub.advertise<sensor_msgs::LaserScan>("/rplidar_scan", 10);
  
  ros::NodeHandle nHandsub;  
  ros::Subscriber sub = nHandsub.subscribe("/scan", 10, &sonarPC2Callback);

  ros::spin();
  return 0;
}
