#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>

Servo myservo;  // 创建舵机对象
ros::NodeHandle nh;

std_msgs::String str_msg; // 创建String消息
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup() {
  myservo.attach(9); // 将舵机控制线连接到D9
  nh.initNode();
  nh.advertise(chatter);
}

void loop() {
  myservo.write(90); // 将舵机设置到90度
  delay(1000);

  myservo.write(0); // 将舵机回到0度
  delay(1000);

  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1000);
}
