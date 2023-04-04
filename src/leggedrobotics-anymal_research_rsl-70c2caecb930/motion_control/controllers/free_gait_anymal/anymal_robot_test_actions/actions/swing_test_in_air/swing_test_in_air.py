#! /usr/bin/env python

class Action(ContinuousAction):

    def __init__(self, relay):
        ContinuousAction.__init__(self, relay)

    def start(self):
        rospy.loginfo('[Swing Test in Air]: Starting motion.')
        self.trigger = TriggerOnFeedback(1, 0.9)
        self.goal = load_action_from_file(get_package_path('anymal_robot_test_actions') \
                 + '/actions/swing_test_in_air/swing_motion.yaml')
        self._send_goal()

    def _feedback_callback(self, feedback):
        ContinuousAction._feedback_callback(self, feedback)
        if not self.trigger.check(self.feedback):
            return
        else:
            self._send_goal()

action = Action(action_loader.execute_steps_relay)
