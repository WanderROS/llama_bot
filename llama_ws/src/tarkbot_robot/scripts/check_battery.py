#!/usr/bin/env python3
# coding:utf-8

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
import os

def battery_callback(msg):
    if msg.data < 11.2:
        rospy.logwarn("current battery : %0.2f V",msg.data)
        rospy.logwarn("System will shutdown soon!!!!")
        # 停止运行小车
        cmd_pub=rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        twist = Twist()
        twist.linear.x=0
        twist.linear.y=0
        twist.linear.z=0

        twist.angular.x=0
        twist.angular.y=0
        twist.angular.z=0

        cmd_pub.publish(twist)
        # 停止激光雷达
        laser_pub=rospy.Publisher('/laser_on_off', Int8, queue_size = 1)
        lase_off = Int8()
        lase_off.data = 0
        laser_pub.publish(lase_off)

        os.system("shutdown -h now")
    else:
        pass


if __name__ == '__main__':
    rospy.init_node('battery_monitor_node', anonymous=True)
    rospy.Subscriber('/bat_vol', Float32, battery_callback)
    rospy.spin()