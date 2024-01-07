# RMC_PROJECT

## ros启动流程

* 文件：rmc_ws/src/DynamixelSDK/ros/dynamixel_sdk_examples/src
* rosrun dynamixel_sdk_examples ...

## AX-12A参数：

```c++
AX-12A参数：
// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting  id先用dynamixel wizard扫描看id
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               9               // DXL2 ID
#define BAUDRATE              1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  
// [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command 根据显示更改usb
```

## 一些注意事项

* 8号电机 真实id其实是1号

##### 电机角度（512是中心） position308 = 90du=0.5pi=1.57079632679       3.422222position = 1du

x * 196.07888989 = position change

1. 稳定的电机:
   * 8（1） 9 12 6 5 1 2
   * 较小范围: 6 9 12
   * 较大范围: 8(1) 1  2
2. 范围从820-204
   * id:  16 9 8(实际是1) 6 12
3. 范围从1024-0
   * id: 14 2 1
4. 没测试的电机： 1 2 3 5

## github

![image-20240103170043179](README.assets/image-20240103170043179.png)



* 抓取姿态：2：512       6：700     12：717
* 转移姿态：



### address of AX-12A

![image-20240104173845224](README.assets/image-20240104173845224.png)

![image-20240104173857311](README.assets/image-20240104173857311.png)

## 使用git

#### 开始使用git

* sudo apt install git
* git config --global user.name "用户名"
* git config --global user.email "电子邮件地址"
* git clone -b master https://github.com/yupose-martin/navigationRobot_ros1.git
* 现在已经有了rmc_ws文件夹
* cd rmc_ws && catkin_make
* 应该就可以使用了

#### 怎样上传

* (在路径～/rmc_ws/进行操作)
* git pull (从远程repository更新自己的文件，一般先pull)
* git add . (将自己更新的文件全部加入这个池子)
* git commit -a -m "你对于此次更新想要说的话"  （-a 是all   -m 是message 在“”里面写入你想要写的信息）（将池子commit）
* git push (push到远程的repository中)

#### 怎样查看commit的更改增减

* git log （可以看到commit记录，附带一个hash码）
* git show 哈希码 （可以清晰的看到该次commit的+ - ）
