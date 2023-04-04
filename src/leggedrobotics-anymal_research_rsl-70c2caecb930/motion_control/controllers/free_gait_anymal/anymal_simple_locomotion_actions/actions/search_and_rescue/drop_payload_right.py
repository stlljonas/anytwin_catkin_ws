#! /usr/bin/env python

from os import listdir
from os.path import *
import any_msgs.srv
import std_srvs.srv
import rocoma_msgs.srv
from actionlib_msgs.msg import GoalStatus


class Action(ActionBase):

    def __init__(self, relay):
        ActionBase.__init__(self, relay)
        self.actions = [self._drop_payload_right,
                        self._straighten_base]
        self.directory = get_package_path('anymal_simple_locomotion_actions') + '/actions' + '/search_and_rescue'

    def start(self):
        self.set_state(ActionState.PENDING)
        self._do_next_action()

    def _done_callback(self, status, result):
        self.result = result
        if status != GoalStatus.SUCCEEDED \
           and status != GoalStatus.PREEMPTED \
           and status != GoalStatus.RECALLED:
            self.set_state(ActionState.ERROR)
        else:
            self._do_next_action()

    def _do_next_action(self):
        self.set_state(ActionState.ACTIVE)
        if self.actions:
            if not self.actions[0]():
                self.set_state(ActionState.ERROR)
            self.actions.pop(0)
        else:
            self.set_state(ActionState.SUCCEEDED)

    # Actions.
    def _drop_payload_right(self):
        rospy.loginfo('drop_payload_right.py: Dropping payload to the right.')
        self.goal = load_action_from_file(self.directory + '/drop_payload_right.yaml')
        self._send_goal()
        return True

    def _straighten_base(self):
        rospy.loginfo('drop_payload_left.py: Straightening base.')
        self.goal = load_action_from_file(self.directory + '/straighten_base.yaml')
        self._send_goal()
        return True

action = Action(action_loader.execute_steps_relay)
