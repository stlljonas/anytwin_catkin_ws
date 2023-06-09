#! /usr/bin/env python

import os
import roslib
roslib.load_manifest('free_gait_action_loader')
from free_gait_action_loader import *
from free_gait import *
import rospy
import actionlib
import free_gait_msgs.msg
import free_gait_msgs.srv
import rospkg
import std_srvs.srv
import traceback
from actionlib_msgs.msg import *
import thread


def packages_to_paths(packages):
    paths = []
    ros_pack = rospkg.RosPack()
    for package in packages:
        if package not in ros_pack.list():
            rospy.logwarn("Cannot find package '%s'." % package)
            continue
        paths.append(ros_pack.get_path(package))
    return paths


def filter_inexistent_paths(paths):
    existent_paths = []
    for path in paths:
        if not os.path.isdir(path):
            rospy.logwarn("Cannot find path '%s'." % path)
            continue
        existent_paths.append(path)
    return existent_paths


class ActionLoader:

    def __init__(self):
        self.name = rospy.get_name()[1:]
        prioritized_free_gait_action_packages = rospy.get_param('~free_gait_action_packages', {})
        prioritized_free_gait_action_paths = rospy.get_param('~free_gait_action_paths', {})
        free_gait_action_packages = [prioritized_free_gait_action_packages[priority] for priority in sorted(prioritized_free_gait_action_packages.keys())]
        free_gait_action_paths = [prioritized_free_gait_action_paths[priority] for priority in sorted(prioritized_free_gait_action_paths.keys())]
        free_gait_action_paths.extend(packages_to_paths(free_gait_action_packages))
        free_gait_action_paths = filter_inexistent_paths(free_gait_action_paths)
        self.action_list = ActionList(self.name, free_gait_action_paths)
        self.action_list.update()
        self.collection_list = CollectionList(self.name, free_gait_action_paths)
        self.collection_list.update()
        self.action_sequence_queue = []
        self.action = None
        self.sending_in_progress = False
        # Reference to the action client or preview publisher.
        self.execute_steps_relay = None

        step_action_server_topic = rospy.get_param('/free_gait/action_server')
        self.execute_steps_client = actionlib.SimpleActionClient(step_action_server_topic, free_gait_msgs.msg.ExecuteStepsAction)

        self.execute_action_server = actionlib.SimpleActionServer("~execute_action", free_gait_msgs.msg.ExecuteActionAction, \
                                                          execute_cb=self._execute_action_callback, auto_start = False)
        self.execute_action_server.register_preempt_callback(self.preempt)
        self.execute_action_server.start()

        step_preview_topic = rospy.get_param('/free_gait/preview_topic')
        self.preview_publisher = rospy.Publisher(step_preview_topic, free_gait_msgs.msg.ExecuteStepsActionGoal, queue_size=1)

        rospy.Service('~update', std_srvs.srv.Trigger, self.update)
        rospy.Service('~list_actions', free_gait_msgs.srv.GetActions, self.list_actions)
        rospy.Service('~list_collections', free_gait_msgs.srv.GetCollections, self.list_collections)
        rospy.Service('~send_action', free_gait_msgs.srv.SendAction, self._send_action_callback)
        rospy.Service('~preview_action', free_gait_msgs.srv.SendAction, self._preview_action_callback)
        rospy.Service('~send_action_sequence', free_gait_msgs.srv.SendActionSequence, self._send_action_sequence_callback)
        rospy.on_shutdown(self.preempt)

    def update(self, request):
        success = self.action_list.update() and self.collection_list.update()
        response = std_srvs.srv.TriggerResponse()
        response.success = success
        return response

    def list_actions(self, request):
        response = free_gait_msgs.srv.GetActionsResponse()
        action_ids = []
        if request.collection_id:
            collection = self.collection_list.get(request.collection_id)
            if collection is None:
                return response
            action_ids = collection.action_ids
        response.actions = self.action_list.to_ros_message(action_ids)
        return response

    def list_collections(self, request):
        response = free_gait_msgs.srv.GetCollectionsResponse()
        response.collections = self.collection_list.to_ros_message()
        return response

    def _execute_action_callback(self, goal):
        rospy.loginfo('Goal has been received: ' + str(goal))
        self.action_sequence_queue = []

        #Start action
        result = free_gait_msgs.msg.ExecuteActionResult()
        result = self.send_action(goal.action_id, False)
        if result.status == result.RESULT_NOT_FOUND:
            self.execute_action_server.set_aborted(result)
            rospy.loginfo('Goal action not found: ' + str(goal))
        elif result.status == result.RESULT_FAILED or result.status == result.RESULT_UNKNOWN:
            self.execute_action_server.set_aborted(result)
            rospy.loginfo('Goal action loading failed: ' + str(goal))

        if self.action is None:  # action is already None, nothing to wait for
            return

        # Undef. wait for action execution
        self.action.timeout = rospy.Duration()
        self.action.wait_for_state([ActionState.ERROR, ActionState.SUCCEEDED, ActionState.PREEMPTED])

        if self.action.state == ActionState.SUCCEEDED:
            rospy.loginfo('Goal has been successfully executed: ' + str(goal))
            result.status = free_gait_msgs.msg.ExecuteActionResult.RESULT_SUCCEEDED
            self.execute_action_server.set_succeeded(result)
        elif self.action.state == ActionState.PREEMPTED:
            rospy.loginfo('Goal has been preempted: ' + str(goal))
            result.status = free_gait_msgs.msg.ExecuteActionResult.RESULT_PREEMPTED
            self.execute_action_server.set_preempted(result)
        else:
            rospy.loginfo('Goal has been aborted: ' + str(goal))
            result.status = free_gait_msgs.msg.ExecuteActionResult.RESULT_FAILED
            self.execute_action_server.set_aborted(result)
        self.action = None

    def _send_action_callback(self, request):
        self.action_sequence_queue = []
        response = free_gait_msgs.srv.SendActionResponse()
        response.result = self.send_action(request.goal.action_id, False)
        return response

    def _send_action_sequence_callback(self, request):
        self.action_sequence_queue = []
        for goal in request.goals:
            self.action_sequence_queue.append(goal.action_id)
        # Start first goal.
        response = free_gait_msgs.srv.SendActionSequenceResponse()
        response.result = self.send_action(self.action_sequence_queue[0], False)
        self.action_sequence_queue.pop(0)
        return response

    def _preview_action_callback(self, request):
        response = free_gait_msgs.srv.SendActionResponse()
        response.result = self.send_action(request.goal.action_id, True)
        return response

    def send_action(self, action_id, use_preview):
        result = free_gait_msgs.msg.ExecuteActionResult()

        if self.sending_in_progress:
            result.status = result.RESULT_FAILED
            rospy.logerr('Action Loader server is busy.')
            return result

        self.sending_in_progress = True
        result = self._send_action(action_id, use_preview)
        self.sending_in_progress = False
        return result

    def _send_action(self, action_id, use_preview):
        self.reset()
        if use_preview:
            self.execute_steps_relay = self.preview_publisher
        else:
            self.execute_steps_relay = self.execute_steps_client

        action_entry = self.action_list.get(action_id)
        result = free_gait_msgs.msg.ExecuteActionResult()

        if action_entry is None:
            rospy.logerr('Action with id "' + action_id + '" does not exists.')
            result.status = result.RESULT_NOT_FOUND
            return result

        if action_entry.file is None:
            rospy.logerr('File for action with id "' + action_id + '" does not exists.')
            result.status = result.RESULT_NOT_FOUND
            return result

        try:
            if action_entry.type == ActionType.YAML:
                self._load_yaml_action(action_entry.file)
            if action_entry.type == ActionType.COMBINED_YAML:
                self._load_combined_yaml_action(action_entry.file)
            elif action_entry.type == ActionType.PYTHON:
                self._load_python_action(action_entry.file)
            elif action_entry.type == ActionType.LAUNCH:
                self._load_launch_action(action_entry.file)

            if self.action is None:
                result.status = result.RESULT_UNKNOWN
                rospy.logerr('An error occurred while reading the action "{}".'.format(action_entry.name))
                return result

            if self.action.state == ActionState.ERROR or self.action.state == ActionState.UNINITIALIZED:
                result.status = result.RESULT_FAILED
                rospy.logerr('An error occurred while loading the action.')
                return result

            self.action.register_callback(self._action_feedback_callback, self._action_done_callback)
            # Wait max. 5 seconds for action to start
            self.action.timeout = rospy.Duration(5)
            self.action.wait_for_state([ActionState.ERROR, ActionState.ACTIVE, ActionState.IDLE, ActionState.INITIALIZED])

            if self.action.state == ActionState.ACTIVE or self.action.state == ActionState.IDLE or self.action.state == ActionState.INITIALIZED:
                result.status = result.RESULT_SUCCEEDED
                rospy.loginfo('Action was successfully started.')
            elif self.action.state == ActionState.ERROR:
                result.status = result.RESULT_FAILED
                rospy.logerr('An error occurred while initializing the action.')
            else:
                result.status = result.RESULT_FAILED
                rospy.logerr('Action initialization returned unexpected state' + ActionState.to_text(self.action.state) + '.')


        except:
            rospy.logerr('An exception occurred while loading the action.')
            result.status = result.RESULT_FAILED
            rospy.logerr(traceback.print_exc())

        return result

    def _load_yaml_action(self, file_path):
        # Load action from YAML file.
        rospy.loginfo('Loading Free Gait action from YAML file "' + file_path + '".')
        goal = load_action_from_file(file_path)
        rospy.logdebug(goal)
        self.action = SimpleAction(self.execute_steps_relay, goal)
        if goal is None:
            rospy.logerr('Could not load action from YAML file.')
            self.action.set_state(ActionState.ERROR)

    def _load_combined_yaml_action(self, file_path):
        # Load combined action from YAML file listing the combination of YAML actions.
        rospy.loginfo('Loading Free Gait action from combined YAML file "' + file_path + '".')
        self.action = CombinedYamlAction(self.execute_steps_relay)
        self.action.set_goal_from_file(file_path)

    def _load_python_action(self, file_path):
        # Load action from Python script.
        rospy.loginfo('Loading Free Gait action from Python script "' + file_path + '".')
        # action_locals = dict()
        # Kind of nasty, but currently only way to make external imports work.
        execfile(file_path, globals(), globals())
        self.action = action

    def _load_launch_action(self, file_path):
        # Load action with external launch file.
        rospy.loginfo('Loading Free Gait action with launch file "' + file_path + '".')
        self.action = LaunchAction(file_path, self.execute_steps_relay)

    @staticmethod
    def _to_action_feedback(action_state):
        if action_state == ActionState.ERROR:
            return free_gait_msgs.msg.ExecuteActionFeedback.STATE_ERROR
        elif action_state == ActionState.UNINITIALIZED:
            return free_gait_msgs.msg.ExecuteActionFeedback.STATE_UNINITIALIZED
        elif action_state == ActionState.INITIALIZED:
            return free_gait_msgs.msg.ExecuteActionFeedback.STATE_INITIALIZED
        elif action_state == ActionState.PENDING:
            return free_gait_msgs.msg.ExecuteActionFeedback.STATE_PENDING
        elif action_state == ActionState.ACTIVE:
            return free_gait_msgs.msg.ExecuteActionFeedback.STATE_ACTIVE
        elif action_state == ActionState.IDLE:
            return free_gait_msgs.msg.ExecuteActionFeedback.STATE_IDLE
        elif action_state == ActionState.SUCCEEDED or action_state == ActionState.PREEMPTED:
            return free_gait_msgs.msg.ExecuteActionFeedback.STATE_FINISHED
        else:
            return None

    def _action_feedback_callback(self):
        if self.action:
            rospy.loginfo('Action switched to state: ' + ActionState.to_text(self.action.state) + '.')
            if self.execute_action_server.is_active():
                feedback = free_gait_msgs.msg.ExecuteActionFeedback()
                feedback.status = self._to_action_feedback(self.action.state)
                self.execute_action_server.publish_feedback(feedback)

    def _action_done_callback(self):
        if self.action:
            rospy.loginfo('Action switched to state: ' + ActionState.to_text(self.action.state) + '.')

    def _check_and_start_action(self):
        try:
            if self.action is not None:
                if self.action.state == ActionState.INITIALIZED:
                    self.action.start()
        except rospy.ROSInterruptException:
            rospy.logerr(traceback.print_exc())
            pass

    def reset(self):
        self.execute_steps_relay = None
        try:
            if (self.action and (self.action.state != ActionState.SUCCEEDED \
                                 and self.action.state != ActionState.PREEMPTED)):
                self.action.stop()
            del self.action
            self.action = None
            # rospy.logwarn('Canceling action.')
        except NameError:
            rospy.logerr(traceback.print_exc())

    def preempt(self):
        rospy.loginfo("Preempt requested: Stopping the action.")
        if self.action:
            self.action.stop()

if __name__ == '__main__':
    try:
        rospy.init_node('free_gait_action_loader')
        action_loader = ActionLoader()
        rospy.loginfo('Ready to load actions from service call.')

        updateRate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # This is required for having the actions run in the main thread
            # (instead through the thread by the service callback).
            action_loader._check_and_start_action()
            updateRate.sleep()
    except rospy.ROSInterruptException:
        pass
