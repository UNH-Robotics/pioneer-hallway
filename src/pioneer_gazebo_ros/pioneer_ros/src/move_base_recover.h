#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include "geometry_msgs/Twist.h"
#include <math.h>

using namespace std ;

class MoveBaseRecover{
  public:
    MoveBaseRecover(ros::NodeHandle) ;
    ~MoveBaseRecover(){}
  private:
    ros::Subscriber subResult ;
    ros::Subscriber subCmdVel ;
    ros::Publisher pubCmdVel ;
    
    ros::Time initialStoppingTime ;
    ros::Time initialRecoveryTime ;
    bool fGoal ;
    bool fTimer ;
    bool fRecovery ;
    double thetaThreshold ;
    double recoveryTheta ;
    double timeThreshold ;
    
    void actionCallback(const actionlib_msgs::GoalStatusArray&) ;
    void cmdVelCallback(const geometry_msgs::Twist&) ;
};

MoveBaseRecover::MoveBaseRecover(ros::NodeHandle nh){
  subResult = nh.subscribe("move_base/status", 10, &MoveBaseRecover::actionCallback, this) ;
  subCmdVel = nh.subscribe("controller_cmd_vel", 10, &MoveBaseRecover::cmdVelCallback, this) ;
  pubCmdVel = nh.advertise<geometry_msgs::Twist>("pioneer/cmd_vel", 10) ;
  
  ros::param::get("move_base/TrajectoryPlannerROS/max_vel_theta",thetaThreshold) ;
  ROS_INFO_STREAM("Recovery behaviour with rotate robot in place at " << thetaThreshold << " rad/s") ;
  timeThreshold = 1.0 ;
  fGoal = false ;
  fTimer = false ;
  fRecovery = false ;
}

void MoveBaseRecover::actionCallback(const actionlib_msgs::GoalStatusArray& msg){
  if (!msg.status_list.empty()){
    if (msg.status_list[0].status == 1)
      fGoal = true ;
    else
      fGoal = false ;
  }
}

void MoveBaseRecover::cmdVelCallback(const geometry_msgs::Twist& msg){
  geometry_msgs::Twist cmd = msg ;
  if (fRecovery){ // recovery mode
    ros::Duration recoveryDuration = ros::Time::now() - initialRecoveryTime ;
    if (recoveryDuration.toSec() < 1.5) // force 1.5 second turnaround
      cmd.angular.z = recoveryTheta ;
    else { // exit recovery mode
      ROS_INFO("Robot exiting recovery mode...") ;
      fRecovery = false ;
      fTimer = false ;
    }
  }
  else if (fGoal){
    if (fabs(msg.linear.x) < 0.001 && fabs(msg.angular.z) < thetaThreshold){
      if (!fTimer){
        initialStoppingTime = ros::Time::now() ;
        fTimer = true ;
      }
      else{
        ros::Duration stoppedTime = ros::Time::now() - initialStoppingTime ;
        if (stoppedTime.toSec() > timeThreshold){
          geometry_msgs::Twist cmd = msg ;
          ROS_INFO("Overriding move_base command velocities to unstick robot in recovery mode...") ;
          fRecovery = true ; // enter recovery mode
          if (msg.angular.z < 0.0)
            recoveryTheta = -thetaThreshold ;
          else
            recoveryTheta = thetaThreshold ;
          cmd.angular.z = recoveryTheta ;
          initialRecoveryTime = ros::Time::now() ;
        }
      }
    }
    else
      fTimer = false ;
  }
  pubCmdVel.publish(cmd) ;
}
