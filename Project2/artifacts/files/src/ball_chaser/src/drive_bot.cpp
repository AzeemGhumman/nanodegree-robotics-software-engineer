#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{

    // ROS_INFO("GoToPositionRequest received - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);

    // std_msgs::Float64 linear_x, angular_z;

    // Check if requested joint angles are in the safe zone, otherwise clamp them
    // std::vector<float> joints_angles = clamp_at_boundaries(req.joint_1, req.joint_2);

    // Publish clamped joint angles to the arm
    // std_msgs::Float64 joint1_angle, joint2_angle;

    // joint1_angle.data = joints_angles[0];
    // joint2_angle.data = joints_angles[1];

    // joint1_pub.publish(joint1_angle);
    // joint2_pub.publish(joint2_angle);

    // Wait 3 seconds for arm to settle
    // ros::Duration(3).sleep();
    
    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = req.linear_x; // 0.5;
    motor_command.angular.z = req.angular_z; // 0.0;
    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);

    
    // Return a response message
    // res.msg_feedback = "Joint angles set - j1: " + std::to_string(joints_angles[0]) + " , j2: " + std::to_string(joints_angles[1]);
    res.msg_feedback = "Robot successfully performed operation";
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Robot is ready to receive commands ...");

    // // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
    // while (ros::ok()) {
    //     // Create a motor_command object of type geometry_msgs::Twist
    //     geometry_msgs::Twist motor_command;
    //     // Set wheel velocities, forward [0.5, 0.0]
    //     motor_command.linear.x = 0.5;
    //     motor_command.angular.z = 0.0;
    //     // Publish angles to drive the robot
    //     motor_command_publisher.publish(motor_command);
    // }

    // Handle ROS communication events
    ros::spin();

    return 0;
}
