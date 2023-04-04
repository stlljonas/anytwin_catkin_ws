#! /usr/bin/env python

from anymal_actions_common import AnymalPythonAction
from anymal_actions_common import trigger_payload_service_call
from free_gait import ActionBase, ActionState, SimpleAction
from free_gait import get_package_path, load_action_from_file
import rospy


class Action(AnymalPythonAction):
    def __init__(self, relay):
        AnymalPythonAction.__init__(self, relay)
        self.actions = [self._switch_xx_to_oo_leg_config,
                        self._reset_payload_to_zero]
        self.directory = get_package_path('anymal_basic_actions') + '/actions/switch_xx_to_oo_leg_config'

    # Actions.
    def _switch_xx_to_oo_leg_config(self):
        if not trigger_payload_service_call("set_to_rest"):
            return False

        self.goal = load_action_from_file(self.directory + '/switch_xx_to_oo_leg_config.yaml')
        self._send_goal()
        self.set_state(ActionState.ACTIVE)
        return True

    def _reset_payload_to_zero(self):
        if not trigger_payload_service_call("set_to_zero"):
            return False
        self.set_state(ActionState.IDLE)
        return True

action = Action(action_loader.execute_steps_relay)
