#! /usr/bin/env python

from anymal_actions_common import AnymalPythonAction
from anymal_actions_common import trigger_payload_service_call, all_feet_in_contact, \
    wait_for_active_controller, set_zero_velocity_update_for_state_estimator
from free_gait import ActionState
from free_gait import get_package_path, load_action_from_file
import rospy


class Action(AnymalPythonAction):

    def __init__(self, relay):
        AnymalPythonAction.__init__(self, relay)
        self.actions = [self._move_down,
                        self._lift_feet,
                        self._reset_payload_to_zero]
        self.directory = get_package_path("anymal_basic_actions") + "/actions/lie_down"

    # Actions.
    def _move_down(self):
        if not wait_for_active_controller("free_gait"):
            rospy.logerr("This action can only be run safely with free_gait.")
            return False

        if not all_feet_in_contact():
            rospy.logerr("All contact states must be closed to lie down.")
            return False

        if not trigger_payload_service_call("set_to_rest"):
            return False

        rospy.loginfo("lie_down_unsafe.py: Moving down.")
        self.goal = load_action_from_file(self.directory + "/move_down.yaml")
        self._send_goal()
        self.set_state(ActionState.ACTIVE)
        return True

    def _lift_feet(self):
        rospy.loginfo("Lifting feet.")
        self.goal = load_action_from_file(self.directory + "/lift_feet.yaml")

        rospy.loginfo("Activating zero velocity updates for state estimator.")
        if not set_zero_velocity_update_for_state_estimator(True):
            rospy.loginfo("Did not activate zero velocity updates for state estimator, assuming simulation.")
            return False
        self._send_goal()
        self.set_state(ActionState.ACTIVE)
        return True

    def _reset_payload_to_zero(self):
        if not trigger_payload_service_call("set_to_zero"):
            return False
        self.set_state(ActionState.IDLE)
        return True


action = Action(action_loader.execute_steps_relay)
