#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ros::Duration(15).sleep();

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  int i=0;
  float x[3] = {1.228,2.57,0.014};
  float y[3] = {-3.34,-0.97,-0.97};
  float w[3] = {-0.48,0.33,-2.58};
  while(ros::ok()) {
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x[i];
    goal.target_pose.pose.position.y = y[i];
    goal.target_pose.pose.orientation.w = w[i];
    i++;
    if(i==3) i=0;
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();//ros::Duration(10));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal reached");
    else
    ROS_INFO("Goal Failed");
    
    ros::Duration(2).sleep();
  
  }

  return 0;
}
