// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
 * $ rosservice call /get_position "id: 1"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 1000}"
 * $ rosservice call /get_position "id: 2"
 *
 * Author: Zerom
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "dynamixel_sdk_examples/GetMoreMotors.h"
#include "dynamixel_sdk_examples/SetMoreMotors.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_PRESENT_POSITION 36
#define ADDR_MOVING_SPEED     32

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               6               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define DXL3_ID               1              // AX-12A ID
#define DXL4_ID               12
#define BAUDRATE              1000000           // Default Baudrate of DYNAMIXEL AX-12A series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler * packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 2);
GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 2);


bool getMoreMotorsPositionCallback(
  dynamixel_sdk_examples::GetMoreMotors::Request & req,
  dynamixel_sdk_examples::GetMoreMotors::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int16_t position1 = 0;
  int16_t position2 = 0;
  int16_t position3 = 0;
  int16_t position4 = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id1);
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", req.id1);
    return 0;
  }

  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id2);
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", req.id2);
    return 0;
  }

  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id3);
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", req.id3);
    return 0;
  }

  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id4);
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", req.id4);
    return 0;
  }

  dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    position1 = groupSyncRead.getData((uint8_t)req.id1, ADDR_PRESENT_POSITION, 2);
    position2 = groupSyncRead.getData((uint8_t)req.id2, ADDR_PRESENT_POSITION, 2);
    position3 = groupSyncRead.getData((uint8_t)req.id3, ADDR_PRESENT_POSITION, 2);
    position4 = groupSyncRead.getData((uint8_t)req.id3, ADDR_PRESENT_POSITION, 2);
    ROS_INFO("getPosition : [POSITION:%d]", position1);
    ROS_INFO("getPosition : [POSITION:%d]", position2);
    ROS_INFO("getPosition : [POSITION:%d]", position3);
    ROS_INFO("getPosition : [POSITION:%d]", position4);
    res.position1 = position1;
    res.position2 = position2;
    res.position3 = position3;
    res.position4 = position4;
    groupSyncRead.clearParam();
    return true;
  } else {
    ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
    groupSyncRead.clearParam();
    return false;
  }
}

void setMoreMotorsPositionCallback(const dynamixel_sdk_examples::SetMoreMotors::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_position1[2];
  uint8_t param_goal_position2[2];
  uint8_t param_goal_position3[2];
  uint8_t param_goal_position4[2];

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint16_t position1 = (uint16_t)msg->position1; // Convert int32 -> uint32
  param_goal_position1[0] = DXL_LOBYTE(DXL_LOWORD(position1));
  param_goal_position1[1] = DXL_HIBYTE(DXL_LOWORD(position1));
  uint16_t position2 = (uint16_t)msg->position2; // Convert int32 -> uint32
  param_goal_position2[0] = DXL_LOBYTE(DXL_LOWORD(position2));
  param_goal_position2[1] = DXL_HIBYTE(DXL_LOWORD(position2));
  uint16_t position3 = (uint16_t)msg->position3; // Convert int32 -> uint32
  param_goal_position3[0] = DXL_LOBYTE(DXL_LOWORD(position3));
  param_goal_position3[1] = DXL_HIBYTE(DXL_LOWORD(position3));
  uint16_t position4 = (uint16_t)msg->position4; // Convert int32 -> uint32
  param_goal_position4[0] = DXL_LOBYTE(DXL_LOWORD(position4));
  param_goal_position4[1] = DXL_HIBYTE(DXL_LOWORD(position4));

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id1, param_goal_position1);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id1);
  }

  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id2, param_goal_position2);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id2);
  }

  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id3, param_goal_position3);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id3);
  }

  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id4, param_goal_position4);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id4);
  }

  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id1, msg->position1);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id2, msg->position2);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id3, msg->position3);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id4, msg->position4);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }

  groupSyncWrite.clearParam();
}

void setMoreMotorsSpeedCallback(const std_msgs::UInt16::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    exit(0);
  }
  uint16_t speed = (uint16_t)msg->data;
    //set moving speed
  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, DXL1_ID, ADDR_MOVING_SPEED, speed, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable moving speed for Dynamixel ID %d", DXL1_ID);
    exit(0);
  } else{
    ROS_INFO("success for setting moving speed for motor: %d",DXL1_ID);
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, DXL2_ID, ADDR_MOVING_SPEED, speed, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable moving speed for Dynamixel ID %d", DXL2_ID);
    exit(0);
  } else{
    ROS_INFO("success for setting moving speed for motor: %d",DXL2_ID);
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, DXL3_ID, ADDR_MOVING_SPEED, speed, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable moving speed for Dynamixel ID %d", DXL3_ID);
    exit(0);
  } else{
    ROS_INFO("success for setting moving speed for motor: %d",DXL3_ID);
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, DXL4_ID, ADDR_MOVING_SPEED, speed, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable moving speed for Dynamixel ID %d", DXL4_ID);
    exit(0);
  } else{
    ROS_INFO("success for setting moving speed for motor: %d",DXL4_ID);
  }
  return;
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL3_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL4_ID);
    return -1;
  }

  //set moving speed
  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, DXL1_ID, ADDR_MOVING_SPEED, (uint16_t)60, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable moving speed for Dynamixel ID %d", DXL1_ID);
    return -1;
  } else{
    ROS_INFO("success for setting moving speed for motor: %d",DXL1_ID);
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, DXL2_ID, ADDR_MOVING_SPEED, (uint16_t)60, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable moving speed for Dynamixel ID %d", DXL2_ID);
    return -1;
  } else{
    ROS_INFO("success for setting moving speed for motor: %d",DXL2_ID);
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, DXL3_ID, ADDR_MOVING_SPEED, (uint16_t)60, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable moving speed for Dynamixel ID %d", DXL3_ID);
    return -1;
  } else{
    ROS_INFO("success for setting moving speed for motor: %d",DXL3_ID);
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, DXL4_ID, ADDR_MOVING_SPEED, (uint16_t)60, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable moving speed for Dynamixel ID %d", DXL4_ID);
    return -1;
  } else{
    ROS_INFO("success for setting moving speed for motor: %d",DXL4_ID);
  }



  ros::init(argc, argv, "more_motors_node");
  ros::NodeHandle nh;
  ros::ServiceServer sync_get_position_srv = nh.advertiseService("/get_more_motors", getMoreMotorsPositionCallback);
  ros::Subscriber sync_set_position_sub = nh.subscribe("/set_more_motors", 10, setMoreMotorsPositionCallback);
  ros::Subscriber subMotorSpeed = nh.subscribe("/motorSpeed",10,setMoreMotorsSpeedCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}
