#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int8.h>

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void send_goal(MoveBaseClient &ac, double x, double y, double w)
{
  move_base_msgs::MoveBaseGoal goal;

  // Set up frame and stamp parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  // Send the goal position and orientation for the robot to reach
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal reached!");
  else
    ROS_INFO("Failed to reach goal for some reason.");
}

void publish_state(ros::Publisher &robot_state_pub, int state) {
    std_msgs::Int8 robot_state; // 0:Initial_State 1:Pickup 2:Drop_Off 3:End_State
    robot_state.data = state;
    robot_state_pub.publish(robot_state);
    ROS_INFO("Robot state %d published", state);
}

int main(int argc, char** argv){

    // Define pick-up & drop-off locations
    double pickup_point[2]  = {1.0, 0.0};
    double dropoff_point[2] = {-1.0, -1.0};
    
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;
    ros::Publisher robot_state_pub = n.advertise<std_msgs::Int8>("robot_state", 1000);
    ROS_INFO("Initializing navigation"); 
  
    // Define robot state position and publish initial state
    
    int state = 0;
    publish_state(robot_state_pub, state);

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 secs for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Send the goal position and orientation for the robot to reach 
    ROS_INFO("Sending pickup goal");   
    send_goal(ac, pickup_point[0], pickup_point[1], 1.0);
    publish_state(robot_state_pub, 1);
    
    // Simulate the picking up
    ros::Duration(5).sleep(); 
    
    // Send the goal position and orientation for the robot to reach 
    ROS_INFO("Sending dropoff goal");
    send_goal(ac, dropoff_point[0], dropoff_point[1], 1.0);
    publish_state(robot_state_pub, 2);

    return 0;
}
