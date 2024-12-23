#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped

rospy.init_node('robot_localization_publisher')

listener = tf.TransformListener()
publisher = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
publish_rate = rospy.Rate(1000) 

while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))

        # Log debug information
        rospy.loginfo("Transform - Translation: {}, Rotation: {}".format(trans, rot))

        # Create a PoseStamped message and fill in the data
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = trans[0]
        pose_msg.pose.position.y = trans[1]
        pose_msg.pose.position.z = trans[2]
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]

        # Publish the PoseStamped message
        publisher.publish(pose_msg)

        # Log an info message
        # rospy.loginfo("Pose published")

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
