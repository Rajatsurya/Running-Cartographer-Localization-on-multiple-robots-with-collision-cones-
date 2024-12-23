#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math
from tf2_ros import Buffer, TransformListener
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
import time
beta_1 = 0.0#tb3_0
beta_2 = 0.0#nons
id_0_yaw = 0.0
id_1_yaw = 0.0
id_2_yaw = 0.0
id_3_yaw = 0.0
v_nons = 0.0
v_0 = 0.0
initial_time_1 = time.time()  # Initialize outside the conditional blocks
initial_time_2 = time.time()  # Initialize outside the conditional blocks
time_array1=[]
time_array2=[]
counter1=0
counter2=0
elapsed_time1=0
elapsed_time_per_epoch1=0
elapsed_time2=0
elapsed_time_per_epoch2=0


def beta_calculator(event):
    global beta_1, beta_2
    try:
        # Wait for the transform
        transform_stamped = buffer.lookup_transform('map', 'tb3_0/base_footprint', rospy.Time(0), rospy.Duration(0.50))

        orientation_quaternion = (
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w
        )
        roll, pitch, beta_1 = euler_from_quaternion(orientation_quaternion)

        transform_stamped = buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(0.50))
        orientation_quaternion = (
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w
        )
        roll, pitch, beta_2 = euler_from_quaternion(orientation_quaternion)

    except Exception as e:
        rospy.logwarn("Failed to lookup transform: {}".format(e)) 

    # rospy.loginfo('beta_1(tb3_0) = {} beta_2(nons) = {}'.format(beta_1, beta_2))
               

def collison_cone_angles(msg):
    # print("Callback executed!")
    global id_0_yaw, id_1_yaw, id_2_yaw, id_3_yaw
    marker = msg  # Since msg is of type Marker, no need to iterate
    if marker.id == 0:
        _, _, id_0_yaw = euler_from_quaternion((
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w
        ))
    elif marker.id == 1:
        _, _, id_1_yaw = euler_from_quaternion((
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w
        ))
    elif marker.id == 2:
        _, _, id_2_yaw = euler_from_quaternion((
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w
        ))
    elif marker.id == 3:
        _, _, id_3_yaw = euler_from_quaternion((
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w
        ))
    # rospy.loginfo('id_0_yaw = {} id_1_yaw = {} id_2_yaw = {} id_3_yaw = {}'.format(id_0_yaw, id_1_yaw, id_2_yaw, id_3_yaw))

def vel_assigner (msg, robot_name):
    global v_nons, v_0

    # Extract linear velocities
    if robot_name == 'no_ns':
        v_nons = msg.linear.x
    elif robot_name == 'tb3_0':
        v_0 = msg.linear.x     
    else:
        rospy.logwarn("Unknown robot_name: {}".format(robot_name))
        return

    # Log the velocities
    # rospy.loginfo('tb3_0 vel = {} nons vel = {}'.format(v_0,v_nons))

def time_calculator():
    global beta_1, beta_2, yaw_1, yaw_2, v_0, v_nons, id_0_yaw, id_1_yaw, id_2_yaw, id_3_yaw, initial_time_1, initial_time_2, time_array1, time_array2, counter1, elapsed_time1, elapsed_time_per_epoch1,counter2, elapsed_time2, elapsed_time_per_epoch2

    yaw_1 = [id_0_yaw, id_1_yaw]
    yaw_2 = [id_2_yaw, id_3_yaw]
    current_elapsed_time = 0.0
    thresh=0.2
 
    if ((min(yaw_1) <= beta_1 <= max(yaw_1)) or (min(yaw_1) <= beta_1 - math.pi <= max(yaw_1)) or (min(yaw_1) <= beta_1 + math.pi <= max(yaw_1))) and v_0 != 0:
        current_time = time.time()
        current_elapsed_time= current_time - initial_time_1
        time_array1.append(current_elapsed_time)
        #print(time_array)
        if counter1>0:
        	if (time_array1[counter1])-time_array1[counter1-1]>thresh:
        		elapsed_time_per_epoch1=elapsed_time1
        		time_array1=[]
        		counter1=0
        		return
        		
        	else:
        		elapsed_time1=elapsed_time_per_epoch1+((time_array1[counter1-1])-time_array1[0])


        #rospy.loginfo('yaw of tb3_0 robot is in between cones for {:.3f} sec'.format(elapsed_time))   
        rospy.loginfo('yaw of tb3_0 robot is in between cones for {:.3f} sec'.format(elapsed_time1)) 
        counter1=counter1+1

    elif ((min(yaw_2) <= beta_2 <= max(yaw_2)) or (min(yaw_2) <= beta_2 - math.pi <= max(yaw_2)) or (min(yaw_2) <= beta_2 + math.pi <= max(yaw_2))) and v_nons != 0:
        current_time = time.time()
        current_elapsed_time = current_time - initial_time_2
        time_array2.append(current_elapsed_time)
        #print(time_array)
        if counter2>0:
        	if (time_array2[counter2])-time_array2[counter2-1]>thresh:
        		elapsed_time_per_epoch2=elapsed_time2
        		time_array2=[]
        		counter2=0
        		return
        		
        	else:
        		elapsed_time2=elapsed_time_per_epoch2+((time_array2[counter2-1])-time_array2[0])

        rospy.loginfo('yaw of nons robot is in between cones for {:.3f} sec'.format(elapsed_time2)) 
        counter2=counter2+1 

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('time_calculator_node')

        # Create a buffer and listener for TF
        buffer = Buffer()
        listener = TransformListener(buffer)

        # Subscribe to relevant topics
        rospy.Subscriber('/cmd_vel', Twist, vel_assigner, callback_args='no_ns')
        rospy.Subscriber('/tb3_0/cmd_vel', Twist, vel_assigner, callback_args='tb3_0')
        rospy.Subscriber('/collision_cone_markers', Marker, collison_cone_angles)

        # Set up a periodic timer for beta calculation
        rospy.Timer(rospy.Duration(0.20), beta_calculator)

        # Set the loop rate (e.g., 1 Hz) for time calculation
        loop_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            time_calculator()
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS shutdown request received. Exiting...")

