#include <ros/ros.h>
#include "dynamixel_sdk_examples/SyncSetPosition.h"
#include <math.h>
int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "sync_publisher");
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher pub = nh.advertise<dynamixel_sdk_examples::SyncSetPosition>("/sync_set_position", 10);

    // Set the loop rate
    ros::Rate loop_rate(10);

    int t = 0;
    int a = 0;
    int step = 15;
    //控制想法：1通过时间 2pid 3梯形 4正弦
    while (ros::ok()) {
        // Create a new message
        dynamixel_sdk_examples::SyncSetPosition msg;
        
        if (a == 0)
        {
            t += step;
            printf("a is 0\n");
            if (t >= 1023)
            {
                a = 1;
            }    
        } else if (a == 1)
        {
            t -= step;
            printf("a is 1\n");
            if (t <= 0)
            {
                a = 0;
            } 
        }   
        

        msg.id1 = 1; // replace DXL1_ID with the actual ID
        msg.position1 = t; // replace with the desired position
        msg.id2 = 9; // replace DXL2_ID with the actual ID
        msg.position2 = 500; // replace with the desired position
        printf("t is : %d a is: %d\n",t,a);
        // Publish the message
        pub.publish(msg);

        // Spin once to let ROS do its thing
        ros::spinOnce();

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
