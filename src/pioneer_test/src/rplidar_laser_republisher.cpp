#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

ros::Publisher pub;

void laserCallback(const sensor_msgs::LaserScan& msg)
{
  sensor_msgs::LaserScan newscan=msg;
  newscan.header.stamp = ros::Time::now();
  newscan.header.frame_id = "laser";
  
  int n_scans;
  n_scans = round((msg.angle_max - msg.angle_min)/msg.angle_increment) + 1;
    
  for (int ii = 0; ii < n_scans; ii++)
  {
    //newscan.ranges[ii] = msg.ranges[n_scans-ii-1];
    newscan.ranges[ii] = msg.ranges[ii];
  }
  
  pub.publish(newscan);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rplidar_laser_republisher");

  ros::NodeHandle nHandpub;
  pub = nHandpub.advertise<sensor_msgs::LaserScan>("/base_scan", 10);
  
  ros::NodeHandle nHandsub;  
  ros::Subscriber sub = nHandsub.subscribe("/rplidar_scan", 10, &laserCallback);

  ros::spin();
  return 0;
}
