#! /usr/bin/env python

# Turtle like crawling.

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from anymal_msgs.msg import AnymalState
from numpy import *
from tf.transformations import *
from free_gait import *

class CrawlingMode:
    BRUTE_FORCE = 0
    WHOLE_BODY = 1

class CrawlingState:
    UNKNOWN = -1
    WAITING_IN_REST = 0
    WAITING_IN_RETRACT = 1
    WAITING_IN_STEP = 2
    ACTIVE_RETRACTING = 4
    ACTIVE_STEPPING = 5


class Action(ContinuousAction):

    def __init__(self, client, directory):

        ActionBase.__init__(self, client, directory)
        self.crawling_mode = CrawlingMode.BRUTE_FORCE
        self.trigger = TriggerOnFeedback(1, 0.5)
        self.timeout = rospy.Duration(10.0)
        self.keep_alive = True

        self.source_frame = 'odom'
        # 0, 1: meter per step, 2: rad per step
        self.velocity = array([0.0, 0.0, 0.0])
        self.crawling_state = CrawlingState.WAITING_IN_REST

        self.quaruped_state_subscriber = rospy.Subscriber("/state_estimator/anymal_state_throttle", AnymalState, self._anymal_state_callback)
        self.joy_subscriber = rospy.Subscriber("/joy_throttle", Joy, self._joy_callback)
        self.twist_subscriber = rospy.Subscriber("/navigation_safety_checker/command_velocity", TwistStamped, self._twist_callback)
        self.timer = rospy.Timer(rospy.Duration(2.0), self._timer_callback)

    def start(self):
        rospy.loginfo('[FreeGaitActionLoader/TurtleCrawl]: Ready to crawl.')
        ContinuousAction.start(self)

    def _done_callback(self, status, result):
        ContinuousAction._done_callback(self, status, result)
        if status != GoalStatus.SUCCEEDED:
            return

        if self.crawling_state == CrawlingState.ACTIVE_RETRACTING:
            self.crawling_state = CrawlingState.WAITING_IN_RETRACT
        elif self.crawling_state == CrawlingState.ACTIVE_STEPPING:
            self.crawling_state = CrawlingState.WAITING_IN_STEP
        self._generate_goal()

    def stop(self):
        rospy.loginfo('[FreeGaitActionLoader/TurtleCrawl]: Stopping.')
        self.quaruped_state_subscriber.unregister();
        self.joy_subscriber.unregister();
        self.twist_subscriber.unregister();
        self.timer.shutdown();

    def _generate_goal(self):
        if self.crawling_state in [CrawlingState.ACTIVE_RETRACTING, CrawlingState.ACTIVE_STEPPING]:
            return
        if self.crawling_state == CrawlingState.UNKNOWN:
            # TODO Check for current state.
            return

        if self.crawling_state == CrawlingState.WAITING_IN_REST:
            steps = self._generate_prepare_motion()
        elif self.crawling_state == CrawlingState.WAITING_IN_RETRACT:
            if all(abs(self.velocity) == 0.0):
                return
            steps = self._generate_step_motion()
        elif self.crawling_state == CrawlingState.WAITING_IN_STEP:
            steps = self._generate_retract_motion()

        self.goal = free_gait_msgs.msg.ExecuteStepsGoal()
        self.goal.steps.extend(steps)
        self._send_goal()

    def _generate_prepare_motion(self):
        self.crawling_state = CrawlingState.ACTIVE_RETRACTING
        goal = load_action_from_file(self.directory + '/stepping_includes/turtle_crawl_prepare.yaml')
        return goal.steps

    def _generate_step_motion(self):
        self.crawling_state = CrawlingState.ACTIVE_STEPPING

        # Get transformation to current robot pose.
        position = self.anymal_state.pose.pose.position
        orientation = self.anymal_state.pose.pose.orientation
        translation = translation_matrix([position.x, position.y, position.z])
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        z_axis = [0, 0, 1]
        rotation = rotation_matrix(yaw, z_axis)
        transform = concatenate_matrices(translation, rotation)

        steps = []

        if (self.crawling_mode == CrawlingMode.BRUTE_FORCE):
            leg_motion_trajectory_goal = load_action_from_file(self.directory + '/stepping_includes/turtle_crawl_step_brute_force.yaml')
            foot_offset = array(self._compute_foot_offset(self.velocity))

            stepPrepare = free_gait_msgs.msg.Step()
            stepPrepare.end_effector_target = leg_motion_trajectory_goal.steps[0].end_effector_target
            for i, end_effector_target in enumerate(stepPrepare.end_effector_target):
                self._add_offset_to_foot_target(stepPrepare.end_effector_target[i].target_position[0].point, [foot_offset[0], 0.0, 0.0])
            steps.append(stepPrepare)

            stepMove = free_gait_msgs.msg.Step()
            stepMove.end_effector_trajectory = leg_motion_trajectory_goal.steps[1].end_effector_trajectory
            # Add velocity.
            for i, end_effector_trajectory in enumerate(stepMove.end_effector_trajectory):
                self._add_offset_to_foot_trajectory(stepMove.end_effector_trajectory[i].trajectory.points[0].transforms[0], foot_offset)
                self._add_offset_to_foot_trajectory(stepMove.end_effector_trajectory[i].trajectory.points[1].transforms[0], -foot_offset)
            # Add to step.
            steps.append(stepMove)

        elif (self.crawling_mode == CrawlingMode.WHOLE_BODY):
            # Leg motion (constant base).
            leg_motion_step = free_gait_msgs.msg.Step()
            const_base_motion = free_gait_msgs.msg.BaseTarget()
            const_base_motion.target = self.anymal_state.pose
            leg_motion_step.base_target.append(const_base_motion)
            leg_motion_trajectory_goal = load_action_from_file(self.directory + '/stepping_includes/turtle_crawl_step_legs_whole_body.yaml')
            leg_motion_step.end_effector_trajectory = leg_motion_trajectory_goal.steps[0].end_effector_trajectory
            # Add velocity.
            for i, end_effector_trajectory in enumerate(leg_motion_step.end_effector_trajectory):
                foot_offset = self._compute_foot_offset(self.velocity)
                foot_offset = [1.2 * foot_offset[0], 0.0, 0.0]
                self._add_offset_to_foot_trajectory(leg_motion_step.end_effector_trajectory[i].trajectory.points[0].transforms[0], foot_offset)
            # Add to step.
            steps.append(leg_motion_step)
            # Base motion.
            base_motion_step = free_gait_msgs.msg.Step()
            base_motion_trajectory_goal = load_action_from_file(self.directory + '/stepping_includes/turtle_crawl_step_base_whole_body.yaml')
            base_motion_step.base_trajectory = base_motion_trajectory_goal.steps[0].base_trajectory
            base_trajectory = base_motion_step.base_trajectory[0].trajectory
            base_trajectory.header.frame_id = self.source_frame
            base_trajectory.joint_names.append('base')
            # Add velocity.
            self.add_offset_to_base(base_trajectory.points[0].transforms[0], 0.5 * self.velocity)
            self.add_offset_to_base(base_trajectory.points[1].transforms[0], self.velocity)
            # Transform.
            transform_transformation(transform, base_trajectory.points[0].transforms[0])
            transform_transformation(transform, base_trajectory.points[1].transforms[0])
            # Add to step.
            steps.append(base_motion_step)

        return steps

    def _compute_foot_offset(self, base_velocity):
        foot_offset = 0.5 * array(base_velocity)
        return foot_offset

    def _get_transform_from_offset(self, offset):
        translation = translation_matrix([offset[0], offset[1], 0.0])
        rotation = rotation_matrix(offset[2], [0, 0, 1])
        return concatenate_matrices(translation, rotation)

    def _generate_retract_motion(self):
        self.crawling_state = CrawlingState.ACTIVE_RETRACTING
        goal = load_action_from_file(self.directory + '/stepping_includes/turtle_crawl_retract.yaml')
        return goal.steps

    def _add_offset_to_foot_trajectory(self, transformation, offset):
        transform = self._get_transform_from_offset(offset)
        transformation_transformed = transform_transformation(transform, transformation)
        transformation.translation = transformation_transformed.translation
        transformation.rotation = transformation_transformed.rotation

    def _add_offset_to_foot_target(self, position, offset):
        transform = self._get_transform_from_offset(offset)
        position_transformed = transform_position(transform, position)
        position.x = position_transformed.x
        position.y = position_transformed.y
        position.z = position_transformed.z

    def add_offset_to_base(self, transformation, offset):
        transformation.translation.x = transformation.translation.x + offset[0]
        transformation.translation.y = transformation.translation.y + offset[1]
        quaternion = quaternion_from_euler(0, 0, offset[2])
        transformation.rotation.x = quaternion[0]
        transformation.rotation.y = quaternion[1]
        transformation.rotation.z = quaternion[2]
        transformation.rotation.w = quaternion[3]

    def _anymal_state_callback(self, anymal_state):
        self.anymal_state = anymal_state

    def _timer_callback(self, event):
        self._generate_goal()

    def _joy_callback(self, joy):
        if len(joy.axes) == 0:
            return
        joy_values = array([joy.axes[1], joy.axes[0], joy.axes[3]])
        for value in nditer(joy_values, op_flags=['readwrite']):
            if abs(value) < 0.1:
                value[...] = 0.0
        self.velocity = array([0.25, 0.07, 0.18]) * joy_values

    def _twist_callback(self, twist):
        pass
        # TODO Switch from joystick to twist. Attention, they publish at the same time!
        #twist_values = array([twist.twist.linear.x, twist.twist.linear.y, twist.twist.angular.z])
        #self.velocity = array([1.0, 1.0, 1.0]) * twist_values

action = Action(action_loader.client, action_loader.directory)
