#include <ros/ros.h>
#include "dynamixel_sdk_examples/SetMoreMotors.h"
#include <iostream>
#include <math.h>
#include <string>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/PointStamped.h>

//arm theta formula; position change = position_per_theta * theta
#define position_per_theta 196.07888989
#define initialPosition 512
//basic length for arm
#define l0 0.095
#define l1 0.093
#define l2 0.233 //0.093 + 夹爪0.14 //电磁铁0.072
// #define closeGripper 120
// #define openGripper 0
//6 2 1 12 小大大小

#define middle_position 512

int color = 0;

struct GoalPosition//arm末端所要到达的位置
{
    double x;
    double y;
    double z;
};

//input x and y
    GoalPosition goal;

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
    if((value <= 204) || (value >= 820))
    {
        return false;
    }
    return true;
}

bool checkLargeMotor(int value)
{
    if((value <= 0) || (value >= 1024))
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
    if (!checkSmallMotor(msg.position1))
    {
        return false;
    } else if (!checkSmallMotor(msg.position4))
    {
        return false;
    } else if (!checkLargeMotor(msg.position2))
    {
        return false;
    } else if (!checkLargeMotor(msg.position3))
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
    msg.id1 = 6; 
    msg.id2 = 2; 
    msg.id3 = 1;
    msg.id4 = 12;
    msg.position4 = 512;
    ThetaArray thetas = getThetaArray(goalPosition);

    msg.position1 = middle_position + (thetas.theta0 * position_per_theta);
    msg.position2 = middle_position + (thetas.theta1 * position_per_theta);
    msg.position3 = middle_position + (thetas.theta2 * position_per_theta);

    return msg;
}

// 回调函数
void poseStampedCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    ROS_INFO("poseStampedCallback");
    if (msg->header.frame_id == "Red")
    {
        color = 0;
    } else if (msg->header.frame_id == "Green")
    {
        color = 1;
    } else if (msg->header.frame_id == "Blue")
    {
        color = 2;
    }

    goal.x = msg->point.x;
    goal.y = msg->point.y;
    goal.z = 0;
}


int main(int argc, char **argv) {
    std_msgs::UInt16 closeGripper;
    closeGripper.data = 180;

    std_msgs::UInt16 openGripper;
    openGripper.data = 0;


    GoalPosition color1Position;
    color1Position.x = 0;
    color1Position.y = 0.2;
    color1Position.z = 0.1;

    GoalPosition color2Position;
    color2Position.x = 0.1;
    color2Position.y = 0.2;
    color2Position.z = 0.1;

    GoalPosition color0Position;
    color0Position.x = 0.25;
    color0Position.y = -0.2;
    color0Position.z = 0.1;

    GoalPosition moveUpPosition;
    moveUpPosition.x = 0.2;
    moveUpPosition.y = 0;
    moveUpPosition.z = 0.2;


    // Initialize the ROS node
    std::string currentState = "initialState";
    ros::init(argc, argv, "more_motors_publisher");
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher pub = nh.advertise<dynamixel_sdk_examples::SetMoreMotors>("/set_more_motors", 10);
    ros::Publisher pubServo = nh.advertise<std_msgs::UInt16>("/servo",10);
    ros::Subscriber subPose = nh.subscribe<geometry_msgs::PointStamped>("/square_detection",10,poseStampedCallback);
    ros::Publisher pubVel = nh.advertise<std_msgs::UInt16>("/motorSpeed",10);
    // Set the loop rate
    ros::Rate loop_rate(10);

    double theta1 = 0;
    double theta2 = 0;
    
    goal.x = 0.2;
    goal.y = 0;
    goal.z = 0.01;

    dynamixel_sdk_examples::SetMoreMotors initial;
    initial.id1 = 6;
    initial.id2 = 2;
    initial.id3 = 1;
    initial.id4 = 12;
    initial.position1 = initial.position2 = initial.position3 = initial.position4 = initialPosition;
    
    //先初始位置
        pub.publish(initial);
        ros::Duration(3.0).sleep();
    while (ros::ok())
    {
        if (currentState == "initialState")//设置初始位置
        {
            pubServo.publish(openGripper);
            ROS_INFO("initialState!");
            pub.publish(initial);
            currentState = "readyToGrasp";
            ros::Duration(3.0).sleep();
        } else if (currentState == "readyToGrasp")//抓取中的预制位置
        {
            moveUpPosition.x = goal.x;
            moveUpPosition.y = goal.y;
            ROS_INFO("readyToGrasp!");
            ROS_INFO("GOAL: x:%f y:%f z:%f ",moveUpPosition.x,moveUpPosition.y,moveUpPosition.z);
            dynamixel_sdk_examples::SetMoreMotors msg = transferMsg(moveUpPosition);
            ROS_INFO("msg: 1position: %d 2position: %d 3position: %d 4position: %d",
                    msg.position1,msg.position2,msg.position3,msg.position4);
            if (checkMotorPosition(msg))
            {
                pub.publish(msg);
            } else if (!checkMotorPosition(msg))
            {
                currentState = "errorState";
                ROS_INFO("wrong input: goal: x: %f y: %f z: %f\nnow error state\n",goal.x, goal.y, goal.z);
            }
            currentState = "toGraspState";
            ros::Duration(3.0).sleep();
        }
        
        else if (currentState == "toGraspState")//去抓取位置
        {
            std_msgs::UInt16 vel;
            vel.data = 30;
            pubVel.publish(vel);
            ROS_INFO("toGraspState!");
            ROS_INFO("GOAL: x:%f y:%f z:%f ",goal.x,goal.y,goal.z);
            dynamixel_sdk_examples::SetMoreMotors msg = transferMsg(goal);
            ROS_INFO("msg: 1position: %d 2position: %d 3position: %d 4position: %d",
                    msg.position1,msg.position2,msg.position3,msg.position4);
            if (checkMotorPosition(msg))
            {
                pub.publish(msg);
            } else if (!checkMotorPosition(msg))
            {
                currentState = "errorState";
                ROS_INFO("wrong input: goal: x: %f y: %f z: %f\nnow error state\n",goal.x, goal.y, goal.z);
            }
            currentState = "closeGripperState";
            ros::Duration(3.0).sleep();
        } else if (currentState == "closeGripperState")//进行抓取
        {
            ROS_INFO("closeGripperState!");
            pubServo.publish(closeGripper);//根据抓取servo的角度调整
            currentState = "moveUpState";
            ros::Duration(3.0).sleep();
        } else if (currentState == "moveUpState")//移动到高处
        {
            std_msgs::UInt16 vel;
            vel.data = 60;
            pubVel.publish(vel);
            ROS_INFO("moveUPState!");
            ROS_INFO("GOAL: x:%f y:%f z:%f ",moveUpPosition.x,moveUpPosition.y,moveUpPosition.z);
            dynamixel_sdk_examples::SetMoreMotors msg = transferMsg(moveUpPosition);
            ROS_INFO("msg: 1position: %d 2position: %d 3position: %d 4position: %d",
                    msg.position1,msg.position2,msg.position3,msg.position4);
            if (checkMotorPosition(msg))
            {
                pub.publish(msg);
            } else if (!checkMotorPosition(msg))
            {
                currentState = "errorState";
                ROS_INFO("wrong input: goal: x: %f y: %f z: %f\nnow error state\n",goal.x, goal.y, goal.z);
            }
            currentState = "placeState";
            ros::Duration(3.0).sleep();
        } else if (currentState == "placeState")//放置位置
        {
            ROS_INFO("placeState!");
            if(color == 0)//颜色0 RED
            {
                //set goal position for color 0
                ROS_INFO("GOAL: x:%f y:%f z:%f ",color0Position.x,color0Position.y,color0Position.z);
                dynamixel_sdk_examples::SetMoreMotors msg = transferMsg(color0Position);
                ROS_INFO("msg: 1position: %d 2position: %d 3position: %d 4position: %d",
                        msg.position1,msg.position2,msg.position3,msg.position4);
                pub.publish(msg);
            } else if (color == 1)//颜色1  GREEN
            {
                //set goal position for color 1
                ROS_INFO("GOAL: x:%f y:%f z:%f ",color1Position.x,color1Position.y,color1Position.z);
                dynamixel_sdk_examples::SetMoreMotors msg = transferMsg(color1Position);
                ROS_INFO("msg: 1position: %d 2position: %d 3position: %d 4position: %d",
                        msg.position1,msg.position2,msg.position3,msg.position4);
                pub.publish(msg);
            } else if (color == 2)//颜色2  BLUE
            {
                //set goal position for color 2
                ROS_INFO("GOAL: x:%f y:%f z:%f ",color2Position.x,color2Position.y,color2Position.z);
                dynamixel_sdk_examples::SetMoreMotors msg = transferMsg(color2Position);
                ROS_INFO("msg: 1position: %d 2position: %d 3position: %d 4position: %d",
                        msg.position1,msg.position2,msg.position3,msg.position4);
                pub.publish(msg);
            }
            currentState = "openGripperState";
            ros::Duration(3.0).sleep();
        } else if (currentState == "openGripperState")
        {
            ROS_INFO("openGripperState!");
            pubServo.publish(openGripper);
            currentState = "initialState";
            ros::Duration(3.0).sleep();
        } else if(currentState == "errorState")
        {
            ROS_INFO("errorState!");
            pub.publish(initial);
            currentState = "initialState";
            ros::Duration(3.0).sleep();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
        return 0;

    // //测试z轴
    // goal.z = 0.25;
    // while(ros::ok())
    // {
    //     goal.z += 0.01;
    //     ROS_INFO("toGraspState!");
    //     ROS_INFO("GOAL: x:%f y:%f z:%f ",goal.x,goal.y,goal.z);
    //     dynamixel_sdk_examples::SetMoreMotors msg = transferMsg(goal);
    //     ROS_INFO("msg: 1position: %d 2position: %d 3position: %d 4position: %d",
    //             msg.position1,msg.position2,msg.position3,msg.position4);
    //     if (checkMotorPosition(msg))
    //     {
    //         pub.publish(msg);
    //     } else if (!checkMotorPosition(msg))
    //     {
    //         currentState = "errorState";
    //         ROS_INFO("wrong input: goal: x: %f y: %f z: %f\nnow error state\n",goal.x, goal.y, goal.z);
    //     }
    //     currentState = "closeGripperState";
    //     ros::Duration(0.5).sleep();
    // }
    // return 0;
}

