#! /usr/bin/env python

#################################################################################
# square_up.py
#
# Brings the robot to Square Up preserving the leg configuration
#
# Creation Date: 2020-06-10
# Author: Fernando Garcia, fgarcia@anybotics.com
# ANYbotics AG
#################################################################################

import math
import rospy
import trajectory_msgs.msg
from anymal_actions_common import SmartAction
from collections import OrderedDict
from anymal_msgs.msg import Contact
from free_gait import ActionState


class Action(SmartAction):
    def __init__(self, receiver):
        SmartAction.__init__(self, receiver)

        self.actions = [self.switch_to_square_up]

    def switch_to_square_up(self):
        if self.has_minimum_allowed_contacts(3):
            self.define_required_steps()
            if any(self.foot_step_required.values()):
                self.add_steps()
            self.set_height()
        else:
            rospy.logerr("[square_up] More than one missing foot contact.")
            return False
        self._send_goal()
        self.set_state(ActionState.ACTIVE)
        return True

    def define_required_steps(self):
        for feet_name, target_position in self.target_feet_positions.items():
            if self.calculate_distance2d(target_position, self.current_feet_positions[feet_name]) > self.footOffsetThreshold or \
                    self.current_feet_contacts[feet_name] != Contact.STATE_CLOSED:
                self.foot_step_required[feet_name] = True

    def add_steps(self):
        # Tackle possible lifted feet first
        for feet_name, is_required in self.foot_step_required.items():
            if self.current_feet_contacts[feet_name] != Contact.STATE_CLOSED:
                self.set_foot_trajectory(feet_name, self.get_trajectory(feet_name))
                self.set_foot_target(feet_name, self.target_feet_positions[feet_name], True, 'triangle')
                self.set_height()
                self.foot_step_required[feet_name] = False

        # Decide step order based on support polygon area
        foot = self.get_first_foot_to_move()
        if foot in self.foot_step_required:
            required_steps = OrderedDict([(foot, self.foot_step_required.pop(foot))])
        else:
            required_steps = OrderedDict([(foot, True)])
        required_steps.update(self.foot_step_required)

        # Tackle the rest
        for feet_name, is_required in required_steps.items():
            if is_required:
                self.set_height()
                self.set_foot_target(feet_name, self.target_feet_positions[feet_name], True, 'triangle')
                self.set_height()
                self.foot_step_required[feet_name] = False

    def get_trajectory(self, feet_name):
        joint_trajectory_point = trajectory_msgs.msg.JointTrajectoryPoint()
        joint_trajectory_point.time_from_start = rospy.Time(3)

        if feet_name == 'LF_LEG' and self.get_leg_configuration('RF_LEG') == 'X':
            joint_trajectory_point.positions = self.defaultJointAngles[feet_name]['X']
        elif feet_name == 'RF_LEG' and self.get_leg_configuration('LF_LEG') == 'X':
            joint_trajectory_point.positions = self.defaultJointAngles[feet_name]['X']
        elif feet_name == 'LH_LEG' and self.get_leg_configuration('RH_LEG') == 'X':
            joint_trajectory_point.positions = self.defaultJointAngles[feet_name]['X']
        elif feet_name == 'RH_LEG' and self.get_leg_configuration('LH_LEG') == 'X':
            joint_trajectory_point.positions = self.defaultJointAngles[feet_name]['X']
        elif feet_name == 'LF_LEG' and self.get_leg_configuration('RF_LEG') == 'O':
            joint_trajectory_point.positions = self.defaultJointAngles[feet_name]['O']
        elif feet_name == 'RF_LEG' and self.get_leg_configuration('LF_LEG') == 'O':
            joint_trajectory_point.positions = self.defaultJointAngles[feet_name]['O']
        elif feet_name == 'LH_LEG' and self.get_leg_configuration('RH_LEG') == 'O':
            joint_trajectory_point.positions = self.defaultJointAngles[feet_name]['O']
        elif feet_name == 'RH_LEG' and self.get_leg_configuration('LH_LEG') == 'O':
            joint_trajectory_point.positions = self.defaultJointAngles[feet_name]['O']
        else:
            joint_trajectory_point.positions = self.defaultJointAngles[feet_name]['X']
        return joint_trajectory_point

    def get_first_foot_to_move(self):
        static_feet = self.get_feet_max_support_polygon()
        return set(self.current_feet_positions.keys()).difference(set(static_feet)).pop()

    def get_feet_max_support_polygon(self):
        current_area = 0.0
        current_comb = list(self.current_feet_positions.keys())[0:2]
        for feet_name in self.current_feet_positions.keys():
            new_comb = self.current_feet_positions.keys()
            new_comb.remove(feet_name)
            new_area = self.support_triangle_area(self.current_feet_positions[new_comb[0]],
                                                  self.current_feet_positions[new_comb[1]],
                                                  self.current_feet_positions[new_comb[2]])
            if new_area > current_area:
                current_area = new_area
                current_comb = new_comb
        return current_comb

    def support_triangle_area(self, p_a, p_b, p_c):
        side_a = self.calculate_distance2d(p_a, p_b)
        side_b = self.calculate_distance2d(p_b, p_c)
        side_c = self.calculate_distance2d(p_c, p_a)
        s = 0.5 * (side_a + side_b + side_c)
        return math.sqrt(s * (s - side_a) * (s - side_b) * (s - side_c))


try:
    action = Action(action_loader.execute_steps_relay)
except Exception as e:
    action = None
