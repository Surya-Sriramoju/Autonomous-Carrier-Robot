#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle nh;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Publisher goal_status_pub = nh.advertise<std_msgs::UInt8>("/goal_status",1);
 

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  std_msgs::UInt8 status;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  //FIRST GOAL
  
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 7.0;
  goal.target_pose.pose.position.y = -2.0;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 2.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Going to first pickup zone!");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("pickup zone reached");
    status.data = 1;
    ros::Duration(5).sleep();
    goal_status_pub.publish(status);
    ROS_INFO("Object pickedup");
  }
  else
  {
    ROS_INFO("Could not reach pickup zone, sed life");
  }
  
  //SECOND GOAL
  goal.target_pose.pose.position.x = -3.0;
  goal.target_pose.pose.position.y = 11.0;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 2.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Going to drop-off zone!");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("drop-off zone reached");
    status.data = 2;
    goal_status_pub.publish(status);
    ROS_INFO("Object dropped-off");
  }
  else
  {
    ROS_INFO("Could not reach drop-off zone, sed life");
  }
    
  ros::Duration(5).sleep();
  

  return 0;
}
