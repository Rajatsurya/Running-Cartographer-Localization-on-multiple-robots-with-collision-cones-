#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_data_callback(msg, robot_name):
    linear_velocity = msg.linear.x
    rospy.loginfo('{} Linear Velocity: {}'.format(robot_name, linear_velocity))

def cmd_vel_data_subscriber():
    rospy.init_node('cmd_vel_sub', anonymous=True)
    rospy.Subscriber('/tb3_0/cmd_vel', Twist, cmd_vel_data_callback, callback_args='tb3_0')
    rospy.Subscriber('/tb3_1/cmd_vel', Twist, cmd_vel_data_callback, callback_args='tb3_1')
    rospy.Subscriber('/tb3_2/cmd_vel', Twist, cmd_vel_data_callback, callback_args='tb3_2')
    rospy.spin()

if __name__ == '__main__':
    try:
        cmd_vel_data_subscriber()
    except rospy.ROSInterruptException:
        pass
