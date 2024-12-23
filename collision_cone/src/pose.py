#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
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

    rospy.loginfo('{} AMCL Pose - Position (x, y): ({}, {}) Orientation (roll, pitch, yaw): ({}, {}, {})'.format(
        robot_name, position_x, position_y, roll, pitch, yaw))

def amcl_pose_subscriber():
    rospy.init_node('amcl_pose_sub', anonymous=True)
    rospy.Subscriber('/tb3_0/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, callback_args='tb3_0')
    rospy.Subscriber('/tb3_1/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, callback_args='tb3_1')
    rospy.Subscriber('/tb3_2/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, callback_args='tb3_2')
    rospy.spin()

if __name__ == '__main__':
    try:
        amcl_pose_subscriber()
    except rospy.ROSInterruptException:
        pass
