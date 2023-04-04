#! /usr/bin/env python

from anymal_actions_common import AnymalPythonAction
from anymal_actions_common import trigger_payload_service_call
from free_gait import ActionState
from free_gait import get_package_path, load_action_from_file


class Action(AnymalPythonAction):
    def __init__(self, relay):
        AnymalPythonAction.__init__(self, relay)
        self.actions = [self._paw_down_high,
                        self._reset_payload_to_zero]
        self.directory = get_package_path("anymal_demo_actions") + "/actions/paw_down_high"

    def _paw_down_high(self):
        if not trigger_payload_service_call("set_to_rest"):
            return False

        self.goal = load_action_from_file(self.directory + "/paw_down_high.yaml")
        self._send_goal()
        self.set_state(ActionState.ACTIVE)
        return True

    def _reset_payload_to_zero(self):
        if not trigger_payload_service_call("set_to_zero"):
            return False
        self.set_state(ActionState.IDLE)
        return True

action = Action(action_loader.execute_steps_relay)
