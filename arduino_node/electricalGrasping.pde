/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;


void servo_cb( const std_msgs::UInt16& cmd_msg){
  if(cmd_msg.data != 0)
  {
    digitalWrite(9,HIGH);
  } else
  {
    digitalWrite(9,LOW);
  }
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  pinMode(9, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  digitalWrite(9,HIGH);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
