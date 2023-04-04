#! /usr/bin/env python

#################################################################################
# smart_action.py
#
# Generic smart action. Allows setting leg trajectories, foot targets and height
#
# Creation Date: 2020-06-10
# Author: Fernando Garcia, fgarcia@anybotics.com
# ANYbotics AG
#################################################################################

import math
import rospy
import free_gait_msgs
import trajectory_msgs
from geometry_msgs.msg import Point
from anymal_actions_common import AnymalPythonAction
from anymal_actions_common import get_feet_contacts, get_joint_positions, get_feet_positions
from collections import OrderedDict
from free_gait import transform_coordinates


class SmartAction(AnymalPythonAction):
    foot_step_required = OrderedDict()
    target_feet_positions = OrderedDict()
    current_feet_contacts = {}
    current_feet_positions = {}
    front_legs = ['LF_LEG', 'RF_LEG']
    hind_legs = ['RH_LEG', 'LH_LEG']
    joint_names = ['HAA', 'HFE', 'KFE']

    def __init__(self, receiver):
        AnymalPythonAction.__init__(self, receiver)
        self.goal = free_gait_msgs.msg.ExecuteStepsGoal()
        self._get_motion_params_from_server()
        self.get_current_feet_positions()
        self.get_current_feet_contacts()
        self.get_target_feet_positions()

    def _get_motion_params_from_server(self):
        self.defaultBaseHeight = rospy.get_param('/free_gait/defaultBaseHeight')
        self.stanceLength = rospy.get_param('/free_gait/stanceLength')
        self.stanceWidth = rospy.get_param('/free_gait/stanceWidth')
        self.defaultJointAngles = rospy.get_param('/free_gait/defaultJointAngles')
        self.footOffsetThreshold = rospy.get_param('/free_gait/footOffsetThreshold')

    def set_foot_target(self, leg_name, foot, do_base_auto=True, step_profile='straight'):
        step = free_gait_msgs.msg.Step()
        if do_base_auto:
            base_motion = free_gait_msgs.msg.BaseAuto()
            base_motion.height = self.defaultBaseHeight
            base_motion.average_linear_velocity = 0.1
            base_motion.average_angular_velocity = 0.1
            step.base_auto.append(base_motion)
        leg_motion = free_gait_msgs.msg.Footstep()
        leg_motion.profile_type = step_profile
        leg_motion.target.header.frame_id = 'odom'
        leg_motion.name = leg_name
        leg_motion.target.point = self.foot_pos_footprint_to_odom(foot)

        step.footstep.append(leg_motion)
        self.goal.steps.append(step)

    def set_height(self, high_support_margin=False):
        base_auto_step = free_gait_msgs.msg.Step()
        base_motion = free_gait_msgs.msg.BaseAuto()
        if high_support_margin:
            base_motion.support_margin = 0.09
        base_motion.height = self.defaultBaseHeight
        base_auto_step.base_auto.append(base_motion)
        self.goal.steps.append(base_auto_step)

    def set_foot_trajectory(self, feet_name, trajectory):
        leg_motion = free_gait_msgs.msg.JointTrajectory()
        leg_motion.name = feet_name
        leg_motion.ignore_contact = True
        leg_motion.trajectory = trajectory_msgs.msg.JointTrajectory()
        leg_motion.trajectory.joint_names = self.joint_names

        joint_trajectory_point = trajectory
        leg_motion.trajectory.points.append(joint_trajectory_point)

        step = free_gait_msgs.msg.Step()
        step.joint_trajectory.append(leg_motion)
        self.goal.steps.append(step)

    def get_target_feet_positions(self):
        self.target_feet_positions = {'LH_LEG': Point(-(self.stanceLength / 2.0), (self.stanceWidth / 2.0), 0.0),
                                      'LF_LEG': Point((self.stanceLength / 2.0), (self.stanceWidth / 2.0), 0.0),
                                      'RH_LEG': Point(-(self.stanceLength / 2.0), -(self.stanceWidth / 2.0), 0.0),
                                      'RF_LEG': Point((self.stanceLength / 2.0), -(self.stanceWidth / 2.0), 0.0)}

    def get_current_feet_positions(self):
        feet_positions = self.get_feet_position_in_frame('footprint')
        self.current_feet_positions = {
            'LF_LEG': Point(feet_positions[0][0], feet_positions[0][1], feet_positions[0][2]),
            'RF_LEG': Point(feet_positions[1][0], feet_positions[1][1], feet_positions[1][2]),
            'LH_LEG': Point(feet_positions[2][0], feet_positions[2][1], feet_positions[2][2]),
            'RH_LEG': Point(feet_positions[3][0], feet_positions[3][1], feet_positions[3][2])}

    def get_current_feet_contacts(self):
        feet_contacts = get_feet_contacts()
        self.current_feet_contacts = {'LF_LEG': feet_contacts[0], 'RF_LEG': feet_contacts[1],
                                      'LH_LEG': feet_contacts[2], 'RH_LEG': feet_contacts[3]}

    def has_minimum_allowed_contacts(self, min_feet_contacts):
        limit = 4 - min_feet_contacts
        not_in_contact = 0
        feet_contacts = get_feet_contacts()
        for in_contact in feet_contacts:
            if not in_contact:
                not_in_contact += 1

        return not_in_contact <= limit

    def is_same_joint_pos(self, current, expected, feet_name, threshold):
        for joint in self.joint_names:
            if abs(current[feet_name[0:3] + joint] - expected[feet_name[0:3] + joint]) > threshold:
                return False
        return True

    def get_leg_configuration(self, feet_name):
        joint_positions = get_joint_positions()
        joint_to_check = joint_positions[feet_name[0:2] + '_HFE']
        if feet_name in self.front_legs:
            return 'X' if joint_to_check < 3.5 else 'O'
        elif feet_name in self.hind_legs:
            return 'X' if joint_to_check > -3.5 else 'O'
        else:
            return 'X'

    def get_feet_position_in_frame(self, frame_id):
        # Get feet position in specified frame
        try:
            feet_positions_in_frame = [transform_coordinates('odom', frame_id, position)[0]
                                       for position in get_feet_positions()]
            return feet_positions_in_frame
        except rospy.exceptions.ROSException:
            rospy.logerr("ANYmal state not available.")
            return False

    def foot_pos_footprint_to_odom(self, foot_pos):
        foot_pos_odom = transform_coordinates('footprint', 'odom', [foot_pos.x, foot_pos.y, foot_pos.z])
        return Point(foot_pos_odom[0][0], foot_pos_odom[0][1], foot_pos_odom[0][2])

    @staticmethod
    def calculate_distance2d(p1, p2):
        return math.sqrt(((p1.x - p2.x) ** 2) + ((p1.y - p2.y) ** 2))
