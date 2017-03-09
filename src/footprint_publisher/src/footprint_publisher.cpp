#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include "footprint_publisher/Footprint.h"
#include <ros/package.h>

geometry_msgs::Polygon footprint;

bool getFootprint(footprint_publisher::Footprint::Request &req,
		    footprint_publisher::Footprint::Response &res) {
  for(auto iter = footprint.points.begin(); iter != footprint.points.end(); ++iter)
    res.footprint.points.push_back(*iter);
  return true;
}

geometry_msgs::Polygon readFootprint(std::string filename) {
  geometry_msgs::Polygon footprint;

  FILE* modelFile = fopen(filename.c_str(), "r+");
  if(modelFile == NULL) {
    ROS_FATAL("Could not open footprint file: %s", filename.c_str());
    exit(1);
  }

  geometry_msgs::Point32 pt;
  while(fscanf(modelFile, "%f %f\n", &pt.x, &pt.y) == 2) {
	  ROS_INFO("loaded: %f  %f", pt.x, pt.y);
    footprint.points.push_back(pt);
  }
  ROS_INFO("Done loading footprint.");
  return footprint;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "footprint_publisher_node");
  ros::NodeHandle n;
  footprint = readFootprint("footprint.txt");

  ros::ServiceServer service = n.advertiseService("footprint", getFootprint);
  ros::spin();
 
  return 0;
}
