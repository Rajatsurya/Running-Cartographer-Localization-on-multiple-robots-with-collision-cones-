#!/usr/bin/env python3

import math
import rospy
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

# Variables to store linear velocities
v_0 = 0.0
v_1 = 0.0
x_0 = 0.0
y_0 = 0.0
x_1 = 0.0
y_1 = 0.0
r = 0.19
beta = 0.0
alpha_1 = 0.0
alpha_2 = 0.0
marker_pub = None 

#initializing all the global variables

def cmd_vel_callback(msg, robot_name):
    global v_0, v_1

    # Update the linear velocity based on the robot_name
    if robot_name == 'tb3_0':
        v_0 = msg.linear.x  #taking the linear velocity of tb3_0 from the topic tb3_0/cmd_vel

    else:
        v_1 = msg.linear.x  #taking the linear velocity of no name space bot from the topic /cmd_vel 

def tf_pose_callback(event):
    global x_0, y_0, x_1, y_1, r, alpha_1, alpha_2, v_0, v_1

    # Initialize beta to a default value
    beta = 0.0

    # Update the position and orientation based on the robot_name
    try:
        # Wait for the transform
        transform_stamped = buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))

        # Extract translation and rotation
        x_1 = transform_stamped.transform.translation.x
        y_1 = transform_stamped.transform.translation.y

        transform_stamped = buffer.lookup_transform('map', 'tb3_0/base_footprint', rospy.Time(0), rospy.Duration(1.0))
        x_0 = transform_stamped.transform.translation.x
        y_0 = transform_stamped.transform.translation.y

    except Exception as e:
        rospy.logwarn("Failed to lookup transform: {}".format(e))

    x = [msg.polygon.points[0].x, msg.polygon.points[1].x, msg.polygon.points[2].x, msg.polygon.points[3].x]
    y = [msg.polygon.points[0].y, msg.polygon.points[1].y, msg.polygon.points[2].y, msg.polygon.points[3].y] 
    dx = [x[i + 1] - x[i] for i in range(3)] + [x[0] - x[3]]
    dy = [y[i + 1] - y[i] for i in range(3)] + [y[0] - y[3]]
    angles = [math.atan2(dy[i], dx[i]) for i in range(4)]
    beta = sum(angles) / len(angles)
    # Calculate the differences in position
    x_diff = x_1 - x_0
    y_diff = y_1 - y_0
    d = math.sqrt((x_diff)**2 + (y_diff)**2)

    # Calculate the angle in radians using arctangent
    x = math.atan2(y_diff, x_diff)

    # Calculate the argument for math.asin() and ensure it's within the valid range
    argument = (2 * r) / d
    argument = max(min(argument, 1.0), -1.0)

    # Calculate theta_1 and theta_2
    theta_1 = x - math.asin(argument)
    theta_2 = x + math.asin(argument)


    # Calculate alpha_1 and alpha_2
    alpha_1 = alpha_1_calculator(v_0, v_1, d, theta_1, beta)
    alpha_2 = alpha_2_calculator(v_0, v_1, d, theta_2, beta)

    rospy.loginfo('Alpha1: {}'.format(alpha_1))
    rospy.loginfo('Alpha2: {}'.format(alpha_2))

    # Return the calculated values
    return d, theta_1, beta, theta_2

def alpha_1_calculator(v_0, v_1, d, theta_1, beta):#this is calculating the inclination of the velocity vector of the robot (tb3_1) wrt tangent 1
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

def alpha_2_calculator(v_0, v_1, d, theta_2, beta):#this is calculating the inclination of the velocity vector of the robot (tb3_0) wrt tangent 2
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

def collision_cone_calculator(): #this is basically the main function which will use all the other pervious functions which were described
    rospy.Subscriber('/tb3_0/cmd_vel', Twist, cmd_vel_callback, callback_args='tb3_0')
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback, callback_args='no_ns')

    # Create a publisher for Marker messages
    marker_pub = rospy.Publisher('/collision_cone_markers', Marker, queue_size=10)#publishing a topic so we can visulize the results on RVIZ

    # Use a timer to repeatedly call tf_pose_callback
    rospy.Timer(rospy.Duration(0.20), tf_pose_callback)

    # Set the rate at which to publish the Marker messages
    rate = rospy.Rate(300)

    while not rospy.is_shutdown():
        # Publish the Marker messages with the calculated alpha values
        publish_markers(marker_pub)

        rate.sleep()

def publish_markers(marker_pub):#the two markers are published 
	global x_0, y_0
	# Create Marker messages for both alpha_1 and alpha_2
	marker1 = create_marker(x_0, y_0, alpha_1, 0)#different Id's are given to the markers as the same topic is being used 
	marker2 = create_marker(x_0, y_0, alpha_2, 1)

	# Publish the Marker messages
	marker_pub.publish(marker1)
	rospy.loginfo("Marker 1 published")
	rospy.sleep(0.1)  # Introduce a delay between publishing two markers
	marker_pub.publish(marker2)
	rospy.loginfo("Marker 2 published")

def create_marker(x, y, alpha, id_0):
    marker = Marker()
    marker.header.frame_id = "map"#set frame ID to map as x_0 y_0 values are wrt map frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "collision_cone"
    marker.id = id_0
    marker.type = Marker.ARROW  # ARROW type markers are used 
    marker.action = Marker.ADD

    # Set the position based on the robot's current position
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.orientation.w = 1.0

    # Set the orientation based on the provided alpha angle
    quaternion = euler_to_quaternion(0, 0, alpha)
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    # Set the scale of the arrow based on your requirements
    marker.scale.x = 2.0  # Adjust the width of the arrow shaft
    marker.scale.y = 0.02  # Adjust the width of the arrow head
    marker.scale.z = 2.0  # Adjust the length of the arrow

    # Set the color of the arrow based on your requirements
    marker.color.a = 0.5
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    return marker



def euler_to_quaternion(roll, pitch, yaw):#this function is written to convert alpha 1 and alpha 2 to quaternions and then use the data to represent it in RVIZ 
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return [qx, qy, qz, qw]

if __name__ == '__main__':
    try:
        rospy.init_node('collision_cone_node_tb3_0')
        buffer = Buffer()
        listener = TransformListener(buffer)
        collision_cone_calculator()
    except rospy.ROSInterruptException:
        pass

#changes to be made to RVIZ
#add two marker objects and let both of the subscribe to the topic published in this code
#same code should be used for tb3_1 but the names of the robot has to be interchanged