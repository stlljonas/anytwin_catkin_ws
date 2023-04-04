#! /usr/bin/env python

from os import listdir
from os.path import *
import any_msgs.srv
import rocoma_msgs.srv
import anymal_msgs.msg
from actionlib_msgs.msg import GoalStatus


class Action(ActionBase):

    def __init__(self, relay):
        ActionBase.__init__(self, relay)
        self.actions = [self._square_up,
                        self._move_down,
                        self._lift_feet]
        self.directory = get_package_path('anymal_basic_actions') + '/actions/lie_down'

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

    def _set_checking_for_state_estimator(self, enable):
        service_name = '/anymal_ctrl_free_gait/check_state_estimator'
        rospy.wait_for_service(service_name)
        try:
            check_state_estimator = rospy.ServiceProxy(service_name, any_msgs.srv.Toggle)
            response = check_state_estimator(enable)
            return response.success
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return False

    def _set_zero_velocity_update_for_state_estimator(self, enable):
        service_name = '/state_estimator/toggle_zero_velocity_updates'
        try:
            rospy.wait_for_service(service_name, 3.0)
            try:
                zero_velocity_updates_for_state_estimator = rospy.ServiceProxy(service_name, any_msgs.srv.Toggle)
                response = zero_velocity_updates_for_state_estimator(enable)
                return response.success
            except rospy.ServiceException, e:
                rospy.logerror("Service call failed: %s"%e)
                return False
        except rospy.ROSException, e:
            rospy.loginfo("lie_down.py: Did not activate zero velocity updates for state estimator, assuming simulation.")
            return True

    def _do_next_action(self):
        self.set_state(ActionState.ACTIVE)
        if self.actions:
            if not self.actions[0]():
                self.set_state(ActionState.ERROR)
            self.actions.pop(0)
        else:
            self.set_state(ActionState.SUCCEEDED)

    # Actions.
    def _square_up(self):
        rospy.loginfo('lie_down.py: Square up.')
        placeholders = {'<target_frame>': 'odom',
                        '<base_height>': 0.47,
                        '<position_x>': 0.38,
                        '<position_x_negative>': -0.38,
                        '<position_y>': 0.24,
                        '<position_y_negative>': -0.24}

        self.goal = load_action_from_file(get_package_path('anymal_basic_actions') \
                                          + '/actions/square_up_template.yaml', placeholders)
        self._send_goal()
        return True

    def _move_down(self):
        service_name = '/anymal_highlevel_controller/get_active_controller'
        rospy.wait_for_service(service_name)
        try:
            get_active_controller = rospy.ServiceProxy(service_name, rocoma_msgs.srv.GetActiveController)
            active_controller = get_active_controller()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return False

        controller_name = "free_gait"
        if active_controller.active_controller != controller_name:
                rospy.logerr('lie_down.py: This action can only be run safely with ' + controller_name + '.')
                return False

        # Check whether all feet have closed contact states.
        try:
            anymal_state = rospy.wait_for_message("/state_estimator/anymal_state",
                                                  anymal_msgs.msg.AnymalState, 1.0)
            for contact in anymal_state.contacts:
                if contact.state != anymal_msgs.msg.Contact.STATE_CLOSED:
                    rospy.logerr('lie_down.py: All contact states must be closed to lie down.')
                    return False
        except rospy.exceptions.ROSException:
            rospy.logerr("lie_down.py: ANYmal state not available.")
            return False

        rospy.loginfo('lie_down.py: Activating zero velocity updates for state estimator.')
        if not self._set_zero_velocity_update_for_state_estimator(True):
            return False

        rospy.loginfo('lie_down.py: Moving down.')
        self.goal = load_action_from_file(self.directory + '/move_down.yaml')
        self._send_goal()
        return True

    def _lift_feet(self):
        rospy.loginfo('lie_down.py: Lifting feet.')
        self.goal = load_action_from_file(self.directory + '/lift_feet.yaml')
        self._send_goal()
        return True


action = Action(action_loader.execute_steps_relay)
