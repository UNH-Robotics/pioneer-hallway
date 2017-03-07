#include <ros/ros.h>
#include "OdomToMap.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_map_frame_publisher") ;

  ros::NodeHandle nHandle ;
  
  OdomToMap OtoM(nHandle) ;
  
  ros::spin();
  return 0;
}
