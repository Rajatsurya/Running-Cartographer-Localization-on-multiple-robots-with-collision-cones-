#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

rospy.init_node('robot_localization_publisher')

listener = tf.TransformListener()
publisher = rospy.Publisher('/robot_pose', PoseWithCovarianceStamped, queue_size=10)
publish_rate = rospy.Rate(10) 

while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))

        # Log debug information
        rospy.loginfo("Transform - Translation: {}, Rotation: {}".format(trans, rot))

        # Create a PoseWithCovarianceStamped message and fill in the data
        pose_with_cov_msg = PoseWithCovarianceStamped()
        pose_with_cov_msg.header.stamp = rospy.Time.now()
        pose_with_cov_msg.header.frame_id = 'map'
        pose_with_cov_msg.pose.pose.position.x = trans[0]
        pose_with_cov_msg.pose.pose.position.y = trans[1]
        pose_with_cov_msg.pose.pose.position.z = trans[2]
        pose_with_cov_msg.pose.pose.orientation.x = rot[0]
        pose_with_cov_msg.pose.pose.orientation.y = rot[1]
        pose_with_cov_msg.pose.pose.orientation.z = rot[2]
        pose_with_cov_msg.pose.pose.orientation.w = rot[3]

        # Set covariance matrix (example values, you should adjust these based on your application)
        covariance_matrix = np.zeros((6, 6), dtype=np.float64)
        np.fill_diagonal(covariance_matrix, 0.1)  # Example: Diagonal elements set to 0.1
        pose_with_cov_msg.pose.covariance = covariance_matrix.flatten().tolist()

        # Publish the PoseWithCovarianceStamped message
        publisher.publish(pose_with_cov_msg)

        # Log an info message
        # rospy.loginfo("Pose with covariance published")

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
