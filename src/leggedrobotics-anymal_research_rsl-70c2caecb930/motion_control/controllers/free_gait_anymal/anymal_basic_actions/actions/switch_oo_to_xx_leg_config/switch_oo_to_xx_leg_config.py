#! /usr/bin/env python

#################################################################################
# switch_oo_to_xx_leg_config.py
#
# Brings the robot from any configuration to XX
#
# Creation Date: 2020-06-10
# Author: Fernando Garcia, fgarcia@anybotics.com
# ANYbotics AG
#################################################################################

import rospy
import trajectory_msgs
from anymal_actions_common import SmartAction
from anymal_actions_common import trigger_payload_service_call
from anymal_msgs.msg import Contact
from free_gait import ActionState


class Action(SmartAction):
    def __init__(self, receiver):
        SmartAction.__init__(self, receiver)

        self.actions = [self.switch_to_xx_conf,
                        self._reset_payload_to_zero]

    def switch_to_xx_conf(self):
        if self.has_minimum_allowed_contacts(3):
            self.define_required_steps()
            if any(self.foot_step_required.values()):
                if not trigger_payload_service_call("set_to_rest"):
                    return False
                self.add_steps()
            self.set_height()
        else:
            rospy.logerr("[switch_oo_to_xx] More than one missing foot contact.")
            return False
        self._send_goal()
        self.set_state(ActionState.ACTIVE)
        return True

    def _reset_payload_to_zero(self):
        if not trigger_payload_service_call("set_to_zero"):
            return False
        self.set_state(ActionState.IDLE)
        return True

    def define_required_steps(self):
        for feet_name, target_position in self.target_feet_positions.items():
            if self.calculate_distance2d(target_position, self.current_feet_positions[feet_name]) > self.footOffsetThreshold or \
                    self.current_feet_contacts[feet_name] != Contact.STATE_CLOSED or self.get_leg_configuration(feet_name) == 'O':
                self.foot_step_required[feet_name] = True

    def add_steps(self):
        # Tackle possible lifted feet first
        for feet_name, is_required in self.foot_step_required.items():
            if self.current_feet_contacts[feet_name] != Contact.STATE_CLOSED:
                self.set_foot_trajectory(feet_name, self.get_trajectory(feet_name))
                self.set_foot_target(feet_name, self.target_feet_positions[feet_name], False)
                self.foot_step_required[feet_name] = False

        # Tackle possible O conf legs with contact
        for feet_name, is_required in self.foot_step_required.items():
            if self.get_leg_configuration(feet_name) == 'O' and self.current_feet_contacts[feet_name] == Contact.STATE_CLOSED:
                self.set_height(True)
                self.set_foot_trajectory(feet_name, self.get_trajectory(feet_name))
                self.set_foot_target(feet_name, self.target_feet_positions[feet_name], False)
                self.foot_step_required[feet_name] = False

        # Tackle the rest
        for feet_name, is_required in self.foot_step_required.items():
            if is_required:
                self.set_height()
                self.set_foot_target(feet_name, self.target_feet_positions[feet_name], True, 'triangle')
                self.foot_step_required[feet_name] = False

    def get_trajectory(self, feet_name):
        joint_trajectory_point = trajectory_msgs.msg.JointTrajectoryPoint()
        joint_trajectory_point.time_from_start = rospy.Time(3)
        joint_trajectory_point.positions = self.defaultJointAngles[feet_name]['X']
        return joint_trajectory_point


try:
    action = Action(action_loader.execute_steps_relay)
except Exception as e:
    action = None
