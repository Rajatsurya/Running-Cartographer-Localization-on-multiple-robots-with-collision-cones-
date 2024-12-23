#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

# Variables to store linear velocities
v_0 = 0.0
v_1 = 0.0
x_0 = 0.0
y_0 = 0.0
x_1 = 0.0
y_1 = 0.0
r = 0.19

def cmd_vel_callback(msg, robot_name):
    global v_0, v_1

    # Update the linear velocity based on the robot_name
    if robot_name == 'tb3_0':
        v_0 = msg.linear.x

    elif robot_name == 'tb3_2':
        v_1 = msg.linear.x

def amcl_pose_callback(msg, robot_name):
    global x_0, y_0, x_1, y_1, r

    # Initialize beta to a default value
    beta = 0.0

    # Update the position and orientation based on the robot_name
    if robot_name == 'tb3_0':
        x_0 = msg.pose.pose.position.x
        y_0 = msg.pose.pose.position.y
    elif robot_name == 'tb3_2':
        x_1 = msg.pose.pose.position.x
        y_1 = msg.pose.pose.position.y

        orientation_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)

        # Assign yaw to the variable 'beta'
        beta = yaw

    # Calculate the differences in position
    x_diff = x_1 - x_0
    y_diff = y_1 - y_0
    d = math.sqrt((x_diff)**2 + (y_diff)**2)

    # Calculate the angle in radians using arctangent
    x = math.atan2(y_diff, x_diff)
    theta_1 = x - math.asin((2 * r) / d)
    theta_2 = x + math.asin((2 * r) / d)

    # Calculate alpha_1
    alpha_1 = alpha_1_calculator(v_0, v_1, d, theta_1, beta)
    alpha_2 = alpha_2_calculator(v_0, v_1, d, theta_2, beta)
    rospy.loginfo('Alpha1: {}'.format(alpha_1))
    rospy.loginfo('Alpha2: {}'.format(alpha_2))

    return d, theta_1, beta, theta_2


def alpha_1_calculator(v_0, v_1, d, theta_1, beta):
    try:
        if v_0 != 0:
            argument = (v_1 / v_0) * math.sin(beta - theta_1)
            if -1 <= argument <= 1:
                alpha_1 = math.asin(argument) + theta_1
            else:
                rospy.logwarn("Invalid argument for arcsine for Alpha1: {}".format(argument))
                alpha_1 = 0  # or any default value if v_0 is 0
        else:
            alpha_1 = 0  # or any default value if v_0 is 0
    except ValueError as e:
        rospy.logerr("Error in alpha_1 calculation: {}".format(e))
        alpha_1 = 0  # or any default value

    return alpha_1

def alpha_2_calculator(v_0, v_1, d, theta_2, beta):
    try:
        if v_0 != 0:
            argument = (v_1 / v_0) * math.sin(beta - theta_2)
            if -1 <= argument <= 1:
                alpha_2 = math.asin(argument) + theta_2
            else:
                rospy.logwarn("Invalid argument for arcsine for Alpha2: {}".format(argument))
                alpha_2 = 0  # or any default value if v_0 is 0
        else:
            alpha_2 = 0  # or any default value if v_0 is 0
    except ValueError as e:
        rospy.logerr("Error in alpha_1 calculation: {}".format(e))
        alpha_2 = 0  # or any default value

    return alpha_2


def collision_cone_calculator():
    rospy.init_node('collision_cone_calculator', anonymous=True)
    rospy.Subscriber('/tb3_0/cmd_vel', Twist, cmd_vel_callback, callback_args='tb3_0')
    rospy.Subscriber('/tb3_2/cmd_vel', Twist, cmd_vel_callback, callback_args='tb3_2')
    rospy.Subscriber('/tb3_0/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, callback_args='tb3_0')
    rospy.Subscriber('/tb3_2/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, callback_args='tb3_2')
    rospy.spin()

if __name__ == '__main__':
    try:
        collision_cone_calculator()
    except rospy.ROSInterruptException:
        pass
