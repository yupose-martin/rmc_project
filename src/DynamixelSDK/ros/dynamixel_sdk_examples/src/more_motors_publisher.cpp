#include <ros/ros.h>
#include "dynamixel_sdk_examples/SetMoreMotors.h"

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "more_motors_publisher");
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher pub = nh.advertise<dynamixel_sdk_examples::SetMoreMotors>("/set_more_motors", 10);

    // Set the loop rate
    ros::Rate loop_rate(10);

    
    int position = 0;
    bool addOrSub = true;
    int step = 10;
    while (ros::ok()) {
        // Create a new message
        dynamixel_sdk_examples::SetMoreMotors msg;

        msg.id1 = 9; // replace DXL1_ID with the actual ID
        msg.id2 = 1; // replace DXL2_ID with the actual ID
        msg.id3 = 16; // replace DXL3_ID with the actual ID

        if (addOrSub)
        {
            position += step;
            if (position >= 1023)
            {
                addOrSub = false;
            }    
        } else if (!addOrSub)
        {
            position -= step;
            if (position <= 0)
            {
                addOrSub = true;
            } 
        } 
        msg.position1 = msg.position2 = msg.position3 = position;

        // Publish the message
        pub.publish(msg);

        // Spin once to let ROS do its thing
        ros::spinOnce();

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
