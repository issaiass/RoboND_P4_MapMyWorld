#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h" // ball_chaser "DriveToTarget" header file

ros::Publisher motor_command_publisher; // ROS::Publisher motor commands;

// handle_drive_request callback function that executes drive_bot service when requested
// Publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
                          ball_chaser::DriveToTarget::Response& res) 
{
    ROS_INFO("DriveToTargetRequest received - linear:%1.2f, angular:%1.2f", (float)req.linear_x, (float)req.angular_z);


    geometry_msgs::Twist motor_command;     // motor_command object of type geometry_msgs::Twist
    motor_command.linear.x = req.linear_x;  // set wheel linear velocity
    motor_command.angular.z = req.angular_z;// set wheel angular velocity
    motor_command_publisher.publish(motor_command); // Publish angles to drive the robot
    // Return a response message
    res.msg_feedback = "Linear and Angular velocities set - linear: " + std::to_string(req.linear_x) + " , angular: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_bot");     // Initialize a ROS node
    ros::NodeHandle n;     // Create a ROS NodeHandle object

    // publishing a message of type geometry_msgs::Twist with queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send commands");

    ros::spin(); // Handle ROS communication events

    return 0;
}
