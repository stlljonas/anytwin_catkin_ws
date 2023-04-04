#! /usr/bin/env python

import tf2_ros
from tf.transformations import *
from numpy import *
from copy import deepcopy
from collections import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class Leg:
    LF = 0
    RF = 1
    LH = 2
    RH = 3

    @staticmethod
    def to_text(leg):
        if leg == Leg.LF:
            return 'LF_LEG'
        elif leg == Leg.RF:
            return 'RF_LEG'
        elif leg == Leg.LH:
            return 'LH_LEG'
        elif leg == Leg.RH:
            return 'RH_LEG'
        else:
            return None

class Direction:
    FORWARD = 0
    BACKWARD = 1

class Action(ContinuousAction):

    def __init__(self, relay):

        ContinuousAction.__init__(self, relay)
        self.footsteps = []
        self.desired_pose = None

        self.map_frame = "odom"
        self.footprint_frame = "footprint"

        self.max_pose_difference = [0.17, 0.08, 0.22]
        self.default_foot_position = dict()
        self.default_foot_position[Leg.LF]  = array([ 0.35,  0.2])
        self.default_foot_position[Leg.RF] = array([ 0.35, -0.2])
        self.default_foot_position[Leg.LH]  = array([-0.35,  0.2])
        self.default_foot_position[Leg.RH] = array([-0.35, -0.2])

        self.gait_pattern = dict()
        self.gait_pattern[Direction.FORWARD] = [Leg.RH, Leg.RF, Leg.LH, Leg.LF]
        self.gait_pattern[Direction.BACKWARD] = deepcopy(self.gait_pattern[Direction.FORWARD])
        self.gait_pattern[Direction.BACKWARD].reverse()

        self.trigger = TriggerOnFeedback(1, 0.0)
        self.pose_subscriber = rospy.Subscriber('~walk/goal_pose', \
                                                PoseStamped, self._pose_callback)
        self.status_publisher = rospy.Publisher('~walk/reached_pose', Bool, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def start(self):
        self.desired_pose = self._get_current_pose()
        rospy.loginfo('[FreeGaitActionLoader/Walk]: Ready to walk.')
        self.set_state(ActionState.IDLE)

    def _feedback_callback(self, feedback):
        ContinuousAction._feedback_callback(self, feedback)
        if self.trigger.check(self.feedback):
            self._update_locomotion()

    def _done_callback(self, status, result):
        rospy.loginfo('[FreeGaitActionLoader/Walk]: Received action result.')
        if status == GoalStatus.SUCCEEDED:
            self.status_publisher.publish(True)
        elif status == GoalStatus.PREEMPTED:
            # Why do we receive this sometimes after we stop tracking goal?!
            return
        else:
            self.status_publisher.publish(False)
        ContinuousAction._done_callback(self, status, result)

    def stop(self):
        rospy.loginfo('[FreeGaitActionLoader/Walk]: Stopping.')
        if self.pose_subscriber:
            self.pose_subscriber.unregister();
        self.status_publisher.unregister();
        ContinuousAction.stop(self)

    def _pose_callback(self, pose):
        # Get desired pose.
        if pose.header.frame_id != self.map_frame:
            rospy.logwarn('[FreeGaitActionLoader/Walk]: Received desired pose in wrong frame.')
            return
        orientation = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation)
        self.desired_pose = array([pose.pose.position.x, pose.pose.position.y, yaw])
        rospy.loginfo('[FreeGaitActionLoader/Walk]: Received pose:')
        rospy.loginfo(self.desired_pose)

        # Generate footsteps.
        start_pose = self._get_current_pose()
        rospy.loginfo('[FreeGaitActionLoader/Walk]: Start pose:')
        print start_pose
        direction = self._determine_direction(start_pose, self.desired_pose)
        rospy.loginfo('[FreeGaitActionLoader/Walk]: Direction:')
        print direction
        poses = self._compute_incremental_poses(start_pose, self.desired_pose)
        rospy.loginfo('[FreeGaitActionLoader/Walk]: Number of poses:')
        print len(poses)
        rospy.loginfo('[FreeGaitActionLoader/Walk]: Poses:')
        print poses
        self.footsteps = self._compute_footsteps(poses, direction)

        # Send first footstep to locomotion controller.
        self._update_locomotion();

    def _get_current_pose(self):
        transform = get_transform(self.footprint_frame, self.map_frame, tf_buffer = self.tf_buffer)
        (roll, pitch, yaw) = euler_from_matrix(transform)
        translation = translation_from_matrix(transform)
        return array([translation[0], translation[1], yaw])

    def _determine_direction(self, start_pose, end_pose):
        transform = self._get_transform_from_planar_pose(start_pose)
        end_position_in_start_frame = self._transform_planar_position(transform, end_pose[:2])
        if end_position_in_start_frame[0] >= 0.0:
            return Direction.FORWARD
        else:
            return Direction.BACKWARD

    def _compute_incremental_poses(self, start_pose, end_pose):
        pose_difference = end_pose - start_pose
        unwrapped_yaw = (pose_difference[2] + pi) % (2 * pi) - pi
        pose_difference[2] = unwrapped_yaw
        print 'pose_difference'
        print pose_difference
        max_difference_index = argmax(absolute(divide(pose_difference, self.max_pose_difference)))
        print 'max_difference_index'
        print max_difference_index
        n_poses = int(abs(pose_difference[max_difference_index]) // self.max_pose_difference[max_difference_index])
        n_poses = n_poses + 1
        pose_increment = pose_difference / n_poses
        print 'pose_increment'
        print pose_increment
        print 'n_poses'
        print n_poses
        poses = [start_pose + pose_increment]
        for i in range(1, n_poses):
            poses.append(poses[i - 1] + pose_increment)
        return poses

    def _get_transform_from_planar_pose(self, pose):
        translation = translation_matrix(append(pose[:2], 0))
        rotation = rotation_matrix(pose[2], (0, 0, 1))
        return concatenate_matrices(translation, rotation)

    def _transform_planar_position(self, transform, position):
        transformed_position = transform.dot(append(position, [0.0, 1.0]))
        return array(transformed_position[:2])

    def _compute_footsteps(self, poses, direction):
        footsteps = []
        for pose in poses:
            transform = self._get_transform_from_planar_pose(pose)
            for leg in self.gait_pattern[direction]:
                position = self._transform_planar_position(transform, self.default_foot_position[leg])
                footsteps.append((leg, position))
        return footsteps

    def _update_locomotion(self):
        if len(self.footsteps) == 0:
            return

        footstep = self.footsteps.pop(0)
        transform = get_transform(self.footprint_frame, self.map_frame, tf_buffer = self.tf_buffer)
        height = transform[2, 3]
        foothold_3d = append(footstep[1], height)

        self.goal = free_gait_msgs.msg.ExecuteStepsGoal()
        prepstep = free_gait_msgs.msg.Step()
        prep_base_motion = free_gait_msgs.msg.BaseAuto()
        prep_base_motion.height = 0.47
        prepstep.base_auto.append(prep_base_motion)
        self.goal.steps.append(prepstep)
        step = free_gait_msgs.msg.Step()
        base_motion = free_gait_msgs.msg.BaseAuto()
        step.base_auto.append(base_motion)
        leg_motion = free_gait_msgs.msg.Footstep()
        leg_motion.name = Leg.to_text(footstep[0]);
        leg_motion.target.header.frame_id = self.map_frame
        leg_motion.target.point = Point(foothold_3d[0], foothold_3d[1], foothold_3d[2])
        step.footstep.append(leg_motion)
        self.goal.steps.append(step)

        if len(self.footsteps) == 0:
            self.goal.steps.append(prepstep)

        self._send_goal()


action = Action(action_loader.execute_steps_relay)
