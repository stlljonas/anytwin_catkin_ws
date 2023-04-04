#! /usr/bin/env python

from anymal_actions_common import AnymalPythonAction
from anymal_actions_common import trigger_payload_service_call, all_feet_in_open_contact, set_checking_for_state_estimator, \
    wait_for_active_controller, wait_for_state_estimator_status, set_zero_velocity_update_for_state_estimator, reset_free_gait_controller, reset_state_estimator_here
from free_gait import ActionState
from free_gait import get_package_path, load_action_from_file
import rospy


class Action(AnymalPythonAction):
    def __init__(self, relay):
        AnymalPythonAction.__init__(self, relay)
        self.actions = [self._create_foot_contact,
                        self._stand_up,
                        self._reset_payload_to_zero]
        self.directory = get_package_path("anymal_c_actions") + "/actions/stand_up"

    # Actions.
    def _create_foot_contact(self):
        if not wait_for_active_controller("free_gait"):
            rospy.logerr("Free gait not running.")
            return False
        
        if not all_feet_in_open_contact():
            rospy.logerr("All contact states must be open to stand up.")
            return False

        if not reset_state_estimator_here():
            rospy.logerr("Could not reset state estimator.")
            return False
        
        if not trigger_payload_service_call("set_to_rest"):
            return False

        rospy.loginfo("Creating foot contact.")
        self.goal = load_action_from_file(self.directory + "/create_foot_contact.yaml")
        self._send_goal()
        self.set_state(ActionState.ACTIVE)

        return True

    def _stand_up(self):
        rospy.loginfo("Resetting Free Gait Controller.")
        if not reset_free_gait_controller():
            return False

        if not wait_for_state_estimator_status(0, 5.0):
            return False

        rospy.loginfo("Enabling checking for estimator.")
        if not set_checking_for_state_estimator(True):
            return False

        rospy.loginfo("Deactivating zero velocity updates for state estimator.")
        if not set_zero_velocity_update_for_state_estimator(False):
            return False

        rospy.loginfo("Standing up.")
        self.goal = load_action_from_file(self.directory + "/stand_up.yaml")
        self._send_goal()
        self.set_state(ActionState.ACTIVE)

        return True

    def _reset_payload_to_zero(self):
        if not trigger_payload_service_call("set_to_zero"):
            return False
        self.set_state(ActionState.IDLE)
        return True


action = Action(action_loader.execute_steps_relay)
