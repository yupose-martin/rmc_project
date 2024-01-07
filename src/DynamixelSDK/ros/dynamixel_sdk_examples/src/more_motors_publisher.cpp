#include <ros/ros.h>
#include "dynamixel_sdk_examples/SetMoreMotors.h"
#include <iostream>
#include <math.h>

//arm theta formula; position change = position_per_theta * theta
#define position_per_theta 196.07888989

//basic length for arm
#define l0 0.09
#define l1 0.093
#define l2 0.198 //0.093 + 0.105

#define middle_position 512


struct GoalPosition//arm末端所要到达的位置
{
    double x;
    double y;
    double z;
};

struct ThetaArray
{
    double theta0;//底部电机的角度
    double theta1;//第一个旋转电机的角度
    double theta2;//第二个旋转电机的角度
};

/**
 * @brief calculate joint angles
 * @param l0 第一个整数
 * @param b 第二个整数
 * 
 * @return int 返回两个整数的和
 */
ThetaArray getThetaArray(GoalPosition goalPosition)
{
    //初始化theta array
    ThetaArray thetas;
    thetas.theta0 = 0;
    thetas.theta1 = 0;
    thetas.theta2 = 0;

    thetas.theta0 = atan2(goalPosition.y,goalPosition.x);

    double down = sqrt(pow(goalPosition.y,2) + pow(goalPosition.x,2));
    double up = goalPosition.z - l0;

    double theta1 = atan2(up,down);
    double theta2 = acos((pow(l1,2)  + (pow(down,2) + pow(up,2)) - pow(l2,2)) 
                    / (2 * l1 * sqrt((pow(down,2) + pow(up,2)))));
    thetas.theta1 = 0.5*M_PI - theta1 - theta2;

    double theta3 = acos((pow(l1,2) + pow(l2,2) - (pow(down,2) + pow(up,2)) ) / (2 * l1 * l2));
    thetas.theta2 = M_PI - theta3;

    return thetas;
}

/**
 * @brief check the goalposition
 * @param goalposition end-effector positon of the arm
 * 
*/
bool checkGoalPosition(GoalPosition goalposition)//底边 min:0.21 max:0.25 高度：l0 + l1
{

    return false;
}

bool checkSmallMotor(int value)
{
    if(value < 204 || value > 820)
    {
        return false;
    }
    return true;
}

bool checkLargeMotor(int value)
{
    if(value < 0 || value > 1024)
    {
        return false;
    }
    return true;
}

/**
 * @brief check the motor position
 * @param goalposition end-effector positon of the arm
 * 
*/
bool checkMotorPosition(dynamixel_sdk_examples::SetMoreMotors msg)//底边 min:0.21 max:0.25 高度：l0 + l1
{// large small small large
    if (!checkSmallMotor(msg.position2))
    {
        return false;
    } else if (!checkSmallMotor(msg.position3))
    {
        return false;
    } else if (!checkLargeMotor(msg.position1))
    {
        return false;
    } else if (!checkLargeMotor(msg.position4))
    {
        return false;
    }
    return true;
}

/**
 * @brief transfer goal position into position msg
 * @param goalPosition xyz world coordinate
 * @return return SetMoreMotors msg
*/
dynamixel_sdk_examples::SetMoreMotors transferMsg(GoalPosition goalPosition)
{
    dynamixel_sdk_examples::SetMoreMotors msg;
    msg.id1 = 2; 
    msg.id2 = 6; 
    msg.id3 = 12;
    msg.id4 = 1;
    msg.position4 = 512;
    ThetaArray thetas = getThetaArray(goalPosition);

    msg.position1 = middle_position - (thetas.theta0 * position_per_theta);
    msg.position2 = middle_position + (thetas.theta1 * position_per_theta);
    msg.position3 = middle_position + (thetas.theta2 * position_per_theta);

    return msg;
}


int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "more_motors_publisher");
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher pub = nh.advertise<dynamixel_sdk_examples::SetMoreMotors>("/set_more_motors", 10);
    // Set the loop rate
    ros::Rate loop_rate(10);

    double theta1 = 0;
    double theta2 = 0;
    //input x and y
    GoalPosition goal;
    goal.x = 0.2;
    goal.y = 0;
    goal.z = 0;
    
    while (ros::ok())
    {
        goal.y += 0.0005;
        goal.x += 0.0005;
        ROS_INFO("GOAL: x:%f y:%f z:%f ",goal.x,goal.y,goal.z);

        dynamixel_sdk_examples::SetMoreMotors msg = transferMsg(goal);
        ROS_INFO("msg: 1position: %d 2position: %d 3position: %d",
                msg.position1,msg.position2,msg.position3);
        if (checkMotorPosition(msg))
        {
            pub.publish(msg);
        } else if (!checkMotorPosition(msg))
        {
            exit(0);
            ROS_INFO("wrong input: goal: x: %f y: %f z: %f\n",goal.x, goal.y, goal.z);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;

    // int position = 0;
    // bool addOrSub = true;
    // int step = 10;
    // while (ros::ok()) {
    //     // Create a new message
    //     dynamixel_sdk_examples::SetMoreMotors msg;

    //     msg.id1 = 2; // replace DXL1_ID with the actual ID
    //     msg.id2 = 6; // replace DXL2_ID with the actual ID
    //     msg.id3 = 12; // replace DXL3_ID with the actual ID
    //     msg.id4 = 1;

    //     if (addOrSub)
    //     {
    //         position += step;
    //         if (position >= 1023)
    //         {
    //             addOrSub = false;
    //         }    
    //     } else if (!addOrSub)
    //     {
    //         position -= step;
    //         if (position <= 0)
    //         {
    //             addOrSub = true;
    //         } 
    //     } 
    //     msg.position1 = msg.position2 = msg.position3 = position;
    //     msg.position4 = 512;
    //     // Publish the message
    //     pub.publish(msg);

    //     // Spin once to let ROS do its thing
    //     ros::spinOnce();

    //     // Sleep to maintain the loop rate
    //     loop_rate.sleep();
    // }

    return 0;
}

