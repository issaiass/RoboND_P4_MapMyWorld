#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define region constants
#define REGION_LT  -1
#define REGION_MID  0
#define REGION_RT   1


ros::ServiceClient client; // Define a global client that can request services

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x  = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");
}

// This function detects in which region is the ball
int identify_region(int w, int h, int i)
{
    int row = i%h;
    int col = i%w;
    int region = 2;
    int left, mid;

    left = w/3;
    mid  = 2*w/3;
    if (col > 0 && col < left) {
        region = -1;
    }
    if (col >= left && col < mid) {
        region = 0;
    }
    if (col >=mid && col < w) {
        region = 1;
    } 
    return region;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    bool pixel_found;
    int region;
    int pos;


    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel) {
            pixel_found = true;
            pos = i;
            break; // found a white pixel, then break
        }
    }
    

    // if no white pixel request a stop when there's no white ball seen by the camera
    if (!pixel_found) {   
        drive_robot(0,0);
        ROS_INFO("Region : NA");
    } else {
        // Then, identify if this pixel falls in the left, mid, or right side of the image
        region = identify_region(img.width, img.height, pos);

        // Depending on the white ball position, call the drive_bot function and pass velocities to it
	switch (region) {
	    case REGION_LT:
	        drive_robot(0, 0.5);

                ROS_INFO("Region : LT");
                break;
	    case REGION_MID:
                drive_robot(0.5, 0);
                ROS_INFO("Region : MID");
                break;
            case REGION_RT:
                drive_robot(0, -0.5);
                ROS_INFO("Region : RT");
                break;
            default:
                ROS_INFO("Region : NA");
		drive_robot(0,0);
		break;
        }
    }

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
