#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  // Change node name simple_navigation_goals to pick_objects
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  // change frame_id from "bask_link" to "map"
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation of the pickup for the robot to reach
  goal.target_pose.pose.position.x = 3.0;
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
	ROS_INFO("The robot has reached the pick up point!");
    ros::Duration(5.0).sleep();
    
    ROS_INFO("Start to go to the drop off point.");
	// A position and orientation of the drop off
	goal.target_pose.pose.position.x = -2.0;
    goal.target_pose.pose.position.y = -1.0;
	goal.target_pose.pose.orientation.w = 1.0; 
    
	// Send the drop off position and orientation for the robot to reach
	ROS_INFO("Sending the drop off point information.");
	ac.sendGoal(goal);
	ac.waitForResult();
    
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
		ROS_INFO("The robot has successfully reached the drop off point!");
	} else {
		ROS_INFO("The robot has failed to reach the desired position.");
	}
  } else {
  	ROS_INFO("The robot has failed to reach any position.");
  }
  return 0;
}