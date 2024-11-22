#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int8.h>

int state;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");
  
  ros::NodeHandle n;
  ros::Publisher robot_state_pub = n.advertise<std_msgs::Int8>("robot_state_pub", 1000);
  
  // Define robot state position and publish initial state
  std_msgs::Int8 robot_state; // 0:Initial_State 1:Pickup 2:Drop_Off 3:End_State
  state = 0;
  robot_state.data = state;
  robot_state_pub.publish(robot_state);  

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  
  for (int i=0; i<2; i++) {

    // set up the frame parameters
    goal.target_pose.header.frame_id = "robot_footprint";
    goal.target_pose.header.stamp = ros::Time::now();
  
    // Define a position and orientation for the robot to reach
    if (i == 0) {
      goal.target_pose.pose.position.x = 1.0;
      goal.target_pose.pose.orientation.w = 1.0;
    } else {
      goal.target_pose.pose.position.x = -1.0;
      goal.target_pose.pose.orientation.w = 1.0;
    }

    // Send the goal position and orientation for the robot to reach
      
    ros::Duration(1, 0).sleep();
    ROS_INFO("Sending goal %d", i);
    ac.sendGoal(goal);
  
    // Wait an infinite time for the results
    ac.waitForResult();
  
    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, robot reached the goal %d!", i);
      if (i == 0) { state = 1; }
      else { state = 2; }
    } else {
      ROS_INFO("The robot failed to navigate to goal %d", i);
      state = 3;
    }
    robot_state.data = state;
    robot_state_pub.publish(robot_state);

    ros::Duration(1, 0).sleep();
  
  }
  
  return 0;
}
