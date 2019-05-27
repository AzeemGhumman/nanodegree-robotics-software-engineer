#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Request robot to move to Pickup location
  goal.target_pose.pose.position.x = 7.5;
  goal.target_pose.pose.position.y = 10.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Robot is travelling to the pickup zone");
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

    // Wait for 5 seconds
    ros::Duration(5).sleep();

    // Request robot to move to Dropoff location
    // goal.target_pose.pose.position.x = -10.0;
    // goal.target_pose.pose.position.y = 6.0;
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    
    ROS_INFO("Robot is travelling to the dropoff zone");
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      // Robot reached dropoff zone
    }
    else {
      ROS_INFO("Unable to get to the dropoff zone");
    } 
  }
  else {
    ROS_INFO("Unable to get to the pickup zone");
  }



  return 0;
}

// TODO: Modify code to include another goal position (one is pickup and other is dropoff)
// TODO: Display message when robot reaches the desired pickup zone
// TODO: Wait 5 seconds
// TODO: Move robot to dropoff zone
// TODO: Display message that it has reached dropoff zone 
