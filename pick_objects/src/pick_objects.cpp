#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int marker = 0;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

//pick up
  n.setParam("marker", marker);
  move_base_msgs::MoveBaseGoal goal_pickup;  

  // set up the frame parameters
  goal_pickup.target_pose.header.frame_id = "map";
  goal_pickup.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  int pickup_x = goal_pickup.target_pose.pose.position.x = 2;
  int pickup_y = goal_pickup.target_pose.pose.position.y = 0;
  goal_pickup.target_pose.pose.orientation.w = 1.0;

  n.setParam("pickup_x", pickup_x);
  n.setParam("pickup_y", pickup_y);

  sleep(8);

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(goal_pickup);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The robot has reached the pickup location");
    sleep(5);
    marker=1;
    n.setParam("marker",marker);
  }
  else
    ROS_INFO("The robot failed to reach the pickup loaction");  

//*************************************************************************  

//drop off
  move_base_msgs::MoveBaseGoal goal_dropoff;  

  // set up the frame parameters
  goal_dropoff.target_pose.header.frame_id = "map";
  goal_dropoff.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  int dropoff_x = goal_dropoff.target_pose.pose.position.x = 3;
  int dropoff_y = goal_dropoff.target_pose.pose.position.y = -2;
  goal_dropoff.target_pose.pose.orientation.w = 1.0;

  n.setParam("dropoff_x", dropoff_x);
  n.setParam("dropoff_y", dropoff_y);

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(goal_dropoff);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The robot has reached the dropofff location");
    marker=2;
    n.setParam("marker",marker);
  }
  else
    ROS_INFO("The robot failed to reach the dropoff loaction");
//***********************************************************************


  return 0;
}