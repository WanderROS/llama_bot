#include "laser_on.h"

/*
 * @功  能  主函数，ROS初始化，调用构造函数初始化
 */
int main(int argc, char **argv)
{
    // ROS初始化 并设置节点名称
    ros::init(argc, argv, "laser_on_node");
    ROS_INFO("Ydliar Laser Turn On  ......  ");
    LaserOn laserOn;
}

LaserOn::LaserOn()
{
    ros::NodeHandle nh;
    nh_ = nh;
    laser_on_off_pub_ = nh.advertise<std_msgs::Int8>("laser_on_off", 10);
    ros::Duration(3).sleep();
    laser_on_off_msgs_.data = 1;
    if (ros::ok())
    {
        laser_on_off_pub_.publish(laser_on_off_msgs_);
        ROS_INFO("Ydliar Laser Turn On Success!");
    }
    ros::spin();
}

LaserOn::~LaserOn()
{
    laser_on_off_msgs_.data = 0;
    laser_on_off_pub_.publish(laser_on_off_msgs_);
    // 提示信息
    ROS_INFO("Ydliar Laser Turn Off....");
}