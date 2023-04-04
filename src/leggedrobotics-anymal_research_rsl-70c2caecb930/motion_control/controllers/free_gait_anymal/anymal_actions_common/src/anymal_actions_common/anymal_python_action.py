#! /usr/bin/env python

#################################################################################
# help_functions.py
#
# ANYmal Python Action base class.
#
# Creation Date: 2020-05-05
# Author: Valentin Yuryev, vyuryev@anybotics.com
# ANYbotics AG
#################################################################################

from actionlib_msgs.msg import GoalStatus
from free_gait import ActionBase, ActionState, SimpleAction


class AnymalPythonAction(ActionBase):
    def __init__(self, relay):
        ActionBase.__init__(self, relay)

    def start(self):
        self.set_state(ActionState.PENDING)
        self._do_next_action()
        while self.state != ActionState.ERROR and self.state != ActionState.PREEMPTED and self.state != ActionState.SUCCEEDED:
            if self.state == ActionState.IDLE:
                self._do_next_action()

    def _done_callback(self, status, result):
        self.result = result
        if status == GoalStatus.SUCCEEDED:
            self.set_state(ActionState.IDLE)
        elif status == GoalStatus.PREEMPTED or status == GoalStatus.RECALLED:
            self.set_state(ActionState.PREEMPTED)
        else:
            self.set_state(ActionState.ERROR)

    def _do_next_action(self):
        if self.actions:
            if not self.actions[0]():
                self.set_state(ActionState.ERROR)
            self.actions.pop(0)
            # Wait for steps sent to finish
            self.wait_for_state([ActionState.IDLE, ActionState.ERROR, ActionState.PREEMPTED])
        else:
            self.set_state(ActionState.SUCCEEDED)
