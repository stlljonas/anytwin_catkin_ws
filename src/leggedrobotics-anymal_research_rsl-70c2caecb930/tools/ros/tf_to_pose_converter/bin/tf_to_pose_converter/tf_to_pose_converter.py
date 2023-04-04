#!/usr/bin/env python

import rospy
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

def get_param(name):
    # Get the parameter, print an error if not available.
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        reason = 'Parameter ' + name + ' not found on server.'
        rospy.logerr(reason)
        rospy.signal_shutdown(reason)

if __name__ == '__main__':
    # Initialize the ROS node.
    rospy.init_node('tf_to_pose_converter')

    # Read the ROS parameters.
    child_frame_id = get_param('~child_frame_id')
    parent_frame_id = get_param('~parent_frame_id')
    target_frame_id = get_param('~target_frame_id')
    pose_topic = get_param('~pose_topic')
    angles_topic = get_param('~angles_topic')
    frequency = get_param('~frequency')

    # Set up tf buffer, publisher and rate.
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose_publisher = rospy.Publisher(pose_topic, geometry_msgs.msg.PoseStamped, queue_size=1)
    angles_publisher = rospy.Publisher(angles_topic, geometry_msgs.msg.Vector3Stamped, queue_size=1)
    rate = rospy.Rate(frequency)

    # Repeat until the node is shut down.
    while not rospy.is_shutdown():
        try:
            # Get the latest transformation from child to parent.
            tf_child_to_parent = tf_buffer.lookup_transform(parent_frame_id, child_frame_id, rospy.Time())
            # Get the latest transformation from parent to target.
            tf_parent_to_target = tf_buffer.lookup_transform(target_frame_id, parent_frame_id, rospy.Time())
            
            # Make sure both transformations have the same time stamp:
            # A) If both stamps are zero, we set them to now.
            if tf_child_to_parent.header.stamp.is_zero() and tf_parent_to_target.header.stamp.is_zero():
                tf_child_to_parent.header.stamp = rospy.rostime.Time.now()
                tf_parent_to_target.header.stamp = rospy.rostime.Time.now()
            # B) If one of the stamps is zero, we set it to the other stamp.
            elif tf_child_to_parent.header.stamp.is_zero():
                tf_child_to_parent.header.stamp = tf_parent_to_target.header.stamp
            elif tf_parent_to_target.header.stamp.is_zero():
                tf_parent_to_target.header.stamp = tf_child_to_parent.header.stamp
            # C) None of the stamps is zero, use the older one of both.
            else:
                if tf_child_to_parent.header.stamp <= tf_parent_to_target.header.stamp:
                    tf_parent_to_target = tf_buffer.lookup_transform(target_frame_id, parent_frame_id, tf_child_to_parent.header.stamp)
                else:
                    tf_child_to_parent = tf_buffer.lookup_transform(parent_frame_id, child_frame_id, tf_parent_to_target.header.stamp)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Skip this update.
            rospy.logdebug('Could not get transformation to convert to pose ...')
            rate.sleep()
            continue

        # Convert the transform to a pose.
        pose = geometry_msgs.msg.PoseStamped()
        pose.header = tf_child_to_parent.header
        pose.pose.position.x = tf_child_to_parent.transform.translation.x
        pose.pose.position.y = tf_child_to_parent.transform.translation.y
        pose.pose.position.z = tf_child_to_parent.transform.translation.z
        pose.pose.orientation.w = tf_child_to_parent.transform.rotation.w
        pose.pose.orientation.x = tf_child_to_parent.transform.rotation.x
        pose.pose.orientation.y = tf_child_to_parent.transform.rotation.y
        pose.pose.orientation.z = tf_child_to_parent.transform.rotation.z

        # Convert the orientation to Euler angles.
        quat_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w] #Workaround while tf2 provides no transformations
        euler_angles = tf_conversions.transformations.euler_from_quaternion(quat_list)
        angles_msg = geometry_msgs.msg.Vector3Stamped()
        angles_msg.header = pose.header
        angles_msg.vector.x = euler_angles[0]
        angles_msg.vector.y = euler_angles[1]
        angles_msg.vector.z = euler_angles[2]

        # Transform the position of the pose into the target frame (rotation only, no translation).
        tf_parent_to_target.transform.translation.x = 0.0
        tf_parent_to_target.transform.translation.y = 0.0
        tf_parent_to_target.transform.translation.z = 0.0
        point = geometry_msgs.msg.PointStamped()
        point.header = pose.header
        point.point = pose.pose.position
        pose.pose.position = tf2_geometry_msgs.do_transform_point(point, tf_parent_to_target).point

        # Publish the pose and sleep.
        pose_publisher.publish(pose)
        angles_publisher.publish(angles_msg)
        rate.sleep()

