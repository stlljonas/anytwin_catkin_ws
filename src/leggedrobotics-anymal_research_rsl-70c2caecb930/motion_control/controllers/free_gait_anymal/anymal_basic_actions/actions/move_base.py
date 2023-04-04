#! /usr/bin/env python

from anymal_msgs.srv import SetBasePose
from anymal_msgs.srv import SetBasePoseResponse
from anymal_msgs.srv import SetBaseHeight
from anymal_msgs.srv import SetBaseHeightResponse


class Action(ContinuousAction):

    def __init__(self, relay):
        ContinuousAction.__init__(self, relay)
        self.set_pose_service = rospy.Service('~move_base/set_pose', SetBasePose, self._set_pose)
        self.set_height_service = rospy.Service('~move_base/set_height', SetBaseHeight, self._set_height)

    def start(self):
        rospy.loginfo('[FreeGaitActionLoader/MoveBase]: Ready to receive desired robot' +
                      'base poses and base heights.')
        self.set_state(ActionState.IDLE)

    def stop(self):
        rospy.loginfo('[FreeGaitActionLoader/MoveBase]: Stopping.')
        self.set_pose_service.shutdown()
        self.set_height_service.shutdown()
        ContinuousAction.stop(self)

    def _set_pose(self, request):
        rospy.loginfo('[FreeGaitActionLoader/MoveBase]: Received move pose request.')
        self.goal = free_gait_msgs.msg.ExecuteStepsGoal()
        step = free_gait_msgs.msg.Step()
        base_motion = free_gait_msgs.msg.BaseTarget()
        base_motion.target = request.pose
        base_motion.average_linear_velocity = 0.1
        step.base_target.append(base_motion)
        self.goal.steps.append(step)
        self._send_goal()
        self.wait_for_state(ActionState.IDLE)
        return SetBasePoseResponse(True)

    def _set_height(self, request):
        rospy.loginfo('[FreeGaitActionLoader/MoveBase]: Received set height request.')
        self.goal = free_gait_msgs.msg.ExecuteStepsGoal()
        step = free_gait_msgs.msg.Step()
        base_motion = free_gait_msgs.msg.BaseAuto()
        base_motion.height = request.height
        base_motion.average_linear_velocity = 0.1
        step.base_auto.append(base_motion)
        self.goal.steps.append(step)

        self._send_goal()
        self.wait_for_state(ActionState.IDLE)
        return SetBaseHeightResponse(True)

action = Action(action_loader.execute_steps_relay)
