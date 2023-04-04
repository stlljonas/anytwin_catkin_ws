#! /usr/bin/env python

#################################################################################
# lie_down_unsafe.py
#
# ANYmal C Lie Down Action.
#
# Creation Date: 2020-05-04
# Author: Aravind Vijayan, avijayan@anybotics.com
# ANYbotics AG
#################################################################################

from anymal_actions_common import AnymalPythonAction
from anymal_actions_common import trigger_payload_service_call, all_feet_in_contact, \
  wait_for_active_controller, set_zero_velocity_update_for_state_estimator
from free_gait import ActionState
from free_gait import get_package_path, load_action_from_file
import rospy


class Action(AnymalPythonAction):

  def __init__(self, relay):
    AnymalPythonAction.__init__(self, relay)

    self.actions = [self._square_up,
                    self._move_down,
                    self._lift_feet,
                    self._reset_payload_to_zero]
    self.directory = get_package_path("anymal_basic_actions") + "/actions/lie_down"

  # Actions.
  def _square_up(self):
    rospy.loginfo('lie_down.py: Square up.')
    execfile(get_package_path('anymal_basic_actions') + '/actions/square_up.py', globals(), globals())
    self.square_up_action = action
    # Wide stance configuration
    self.square_up_action.stanceLength = 0.84
    self.square_up_action.stanceWidth = 0.6
    # Re-compute steps for squaring up.
    self.square_up_action.get_target_feet_positions()
    # Perform square up.
    self.square_up_action.switch_to_square_up()
    # Wait for 10 seconds for the action to be completed.
    self.square_up_action.timeout = rospy.Duration(10)
    self.square_up_action.wait_for_state([ActionState.IDLE, ActionState.ERROR, ActionState.PREEMPTED, ActionState.SUCCEEDED])
    # If the square up action was preempted or recalled, set the lie down action to preempted state.
    # If the square up action returns an error, set the lie down action to error state.
    # This avoids the rest of the actions from executing.
    if self.square_up_action.state == ActionState.SUCCEEDED:
      self.set_state(ActionState.IDLE)
    elif self.square_up_action.state == ActionState.IDLE:
      self.set_state(ActionState.IDLE)
    elif self.square_up_action.state == ActionState.PREEMPTED:
      self.set_state(ActionState.PREEMPTED)
    else:
      self.set_state(ActionState.ERROR)

    return True

  def _move_down(self):
    if not wait_for_active_controller("free_gait"):
      rospy.logerr("This action can only be run safely with free_gait.")
      return False

    if not all_feet_in_contact():
      rospy.logerr("All contact states must be closed to lie down.")
      return False

    if not trigger_payload_service_call("set_to_rest"):
      return False

    self.goal = load_action_from_file(self.directory + "/move_down.yaml")
    self._send_goal()
    self.set_state(ActionState.ACTIVE)
    return True

  def _lift_feet(self):
    self.goal = load_action_from_file(self.directory + "/lift_feet.yaml")

    rospy.logdebug("Activating zero velocity updates for state estimator.")
    if not set_zero_velocity_update_for_state_estimator(True):
      rospy.logerror("Could not set zero velocity updates for state estimator. Aborting lie-down.")
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
