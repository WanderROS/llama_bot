#ifndef LASER_ON_H
#define LASER_ON_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
class LaserOn
{
public:
    LaserOn();  // 构造函数
    ~LaserOn(); // 析构函数
private:
    ros::NodeHandle nh_;
    // 发布器定义
    ros::Publisher laser_on_off_pub_;
    std_msgs::Int8 laser_on_off_msgs_;  //激光雷达发布消息
};

#endif