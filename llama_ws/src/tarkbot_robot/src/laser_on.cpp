#include "laser_on.h"
#include <csignal>

bool flg_exit = false;
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

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch CTRL-C Signal!");
}

LaserOn::LaserOn()
{
    ros::NodeHandle nh;
    nh_ = nh;
    signal(SIGINT, SigHandle);
    laser_on_off_pub_ = nh.advertise<std_msgs::Int8>("laser_on_off", 10);
    ros::Duration(3).sleep();
    laser_on_off_msgs_.data = 1;
    if (ros::ok())
    {
        laser_on_off_pub_.publish(laser_on_off_msgs_);
        ROS_INFO("Ydliar Laser Turn On Success!");
    }
    // 捕获到 Ctrl-C 后关闭激光雷达
    ros::Rate rate(1000);
    while (ros::ok())
    {
        if (flg_exit)
        {
            int i = 0;
            while (i < 10)
            {
                i++;
                laser_on_off_msgs_.data = 0;
                laser_on_off_pub_.publish(laser_on_off_msgs_);
                rate.sleep();
            }
            // 提示信息
            ROS_INFO("Ydliar Laser Turn Off....");
            ros::shutdown();
        }
        rate.sleep();
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
