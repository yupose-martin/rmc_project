#include <Servo.h>

Servo myServo;  // 创建舵机对象
int i = 0;
int value = 1;
void setup() {
  Serial.begin(9600);
  myServo.attach(9);  // 将舵机控制线连接到D9
}

void loop() {
  Serial.println(i);
  i += value;
  if(i < 0){
    value = 1;
  }
  if(i > 180)
  {
    value = -1;
  }
  myServo.write(i);  // 将舵机转到90度
  delay(5);
  // delay(100);
  // myServo.write(0);   // 将舵机转到0度
  // delay(1000);
}
