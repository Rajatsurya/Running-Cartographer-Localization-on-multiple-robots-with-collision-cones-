#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf.transformations import euler_from_quaternion

def amcl_pose_callback(msg, robot_name):
    position_x = msg.pose.pose.position.x
    position_y = msg.pose.pose.position.y
    orientation_quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)

    rospy.loginfo('{} AMCL Pose - Position (x, y): ({}, {}) Orientation (yaw): ({})'.format(
        robot_name, position_x, position_y, yaw))

def cmd_vel_data_callback(msg, robot_name):
    linear_velocity = msg.linear.x
    rospy.loginfo('{} Linear Velocity: {}'.format(robot_name, linear_velocity))

def amcl_pose_vel_subscriber():
    rospy.init_node('amcl_pose_vel_sub', anonymous=True)
    rospy.Subscriber('/tb3_0/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, callback_args='tb3_0')
    # rospy.Subscriber('/tb3_1/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, callback_args='tb3_1')
    rospy.Subscriber('/tb3_2/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, callback_args='tb3_2')
    rospy.Subscriber('/tb3_0/cmd_vel', Twist, cmd_vel_data_callback, callback_args='tb3_0')
    # rospy.Subscriber('/tb3_1/cmd_vel', Twist, cmd_vel_data_callback, callback_args='tb3_1')
    rospy.Subscriber('/tb3_2/cmd_vel', Twist, cmd_vel_data_callback, callback_args='tb3_2')
    rospy.spin()

if __name__ == '__main__':
    try:
        amcl_pose_vel_subscriber()
    except rospy.ROSInterruptException:
        pass
