#include <ros/ros.h>
#include "move_base_recover.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base_recover") ;

  ros::NodeHandle nHandle ;
  
  MoveBaseRecover recover(nHandle) ;
  
  ros::spin();
  return 0;
}
