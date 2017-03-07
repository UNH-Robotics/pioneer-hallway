#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

ros::Publisher pub ;
double linear_gains[3], angular_gains[3] ;

void cmd_velCallback(const geometry_msgs::Twist& msg)
{
    geometry_msgs::Twist current_cmd_vel = msg ;
    geometry_msgs::Twist pid_cmd ;
    
//    std::cout << linear_gains[0] << "  " << angular_gains[0] << std::endl ;
    
    pid_cmd.linear.x = current_cmd_vel.linear.x*linear_gains[0] ;
    pid_cmd.linear.y = current_cmd_vel.linear.y*linear_gains[0] ;
    pid_cmd.linear.z = current_cmd_vel.linear.z*linear_gains[0] ;
    pid_cmd.angular.x = current_cmd_vel.angular.x*angular_gains[0] ;
    pid_cmd.angular.y = current_cmd_vel.angular.y*angular_gains[0] ;
    pid_cmd.angular.z = current_cmd_vel.angular.z*angular_gains[0] ;
    
    pub.publish(pid_cmd) ;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pioneer_pid_controller");

    double temp_gain = 0;
    ros::param::get("pioneer_ros/controller_gains/body/linear/p", temp_gain);
    linear_gains[0] = temp_gain;
    ros::param::get("pioneer_ros/controller_gains/body/linear/i", temp_gain);
    linear_gains[1] = temp_gain;
    ros::param::get("pioneer_ros/controller_gains/body/linear/d", temp_gain);
    linear_gains[2] = temp_gain;
    ros::param::get("pioneer_ros/controller_gains/body/angular/p", temp_gain);
    angular_gains[0] = temp_gain;
    ros::param::get("pioneer_ros/controller_gains/body/angular/i", temp_gain);
    angular_gains[1] = temp_gain;
    ros::param::get("pioneer_ros/controller_gains/body/angular/d", temp_gain);
    angular_gains[2] = temp_gain;
    
    ros::NodeHandle nHandpub ;
    pub = nHandpub.advertise<geometry_msgs::Twist>("pioneer/cmd_vel", 10) ;
    
    ros::NodeHandle nHandsub ;  
    ros::Subscriber sub = nHandsub.subscribe("cmd_vel", 10, &cmd_velCallback) ;
    
    ros::spin() ;
    return 0;
}
