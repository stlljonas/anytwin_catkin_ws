/*
 * FreeGaitActionClient.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Georg Wiedebach, Péter Fankhauser
 */

#include <free_gait_ros/FreeGaitActionClient.hpp>

namespace free_gait {

FreeGaitActionClient::FreeGaitActionClient(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      state_(ActionState::UNINITIALIZED)
{
  // Get action server topic.
  std::string actionServerTopic;
  if (nodeHandle_.hasParam("/free_gait/action_server")) {
    nodeHandle_.getParam("/free_gait/action_server", actionServerTopic);
  } else {
    throw std::runtime_error("Did not find ROS parameter for Free Gait Action Server '/free_gait/action_server'.");
  }

  // Get preview topic.
  std::string previewTopic;
  if (nodeHandle_.hasParam("/free_gait/preview_topic")) {
    nodeHandle_.getParam("/free_gait/preview_topic", previewTopic);
  } else {
    throw std::runtime_error(
        "Did not find ROS parameter for Free Gait Preview Topic '/free_gait/preview_topic'.");
  }

  // Get state topic.
  std::string actionStateTopic;
  if (nodeHandle_.hasParam("/free_gait/action_state_topic")) {
    nodeHandle_.getParam("/free_gait/action_state_topic", actionStateTopic);
  } else {
    throw std::runtime_error(
        "Did not find ROS parameter for Free Gait Action State Topic '/free_gait/action_state_topic'.");
  }

  // Initialize action client.
  client_.reset(new actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>(nodeHandle, actionServerTopic));

  // Initialize preview publisher.
  previewPublisher_ = nodeHandle_.advertise<free_gait_msgs::ExecuteStepsActionGoal>(previewTopic, 1, false);

  // Initialize state publisher. State is published every time its set with setState();
  statePublisher_ = nodeHandle_.advertise<free_gait_msgs::ExecuteActionFeedback>(actionStateTopic, 1, true);

  setState(state_ = ActionState::INITIALIZED);
}

void FreeGaitActionClient::registerCallback(
    std::function<void()> activeCallback,
    std::function<void(const free_gait_msgs::ExecuteStepsFeedbackConstPtr&)> feedbackCallback,
    std::function<void(const actionlib::SimpleClientGoalState&,
                       const free_gait_msgs::ExecuteStepsResult&)> doneCallback)
{
  activeCallback_ = activeCallback;
  feedbackCallback_ = feedbackCallback;
  doneCallback_ = doneCallback;
}

void FreeGaitActionClient::sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal)
{
  bool usePreview = false;
  if (!nodeHandle_.hasParam("use_preview")) {
    ROS_DEBUG("Did not find ROS parameter for 'use_preview'. Setting to false.");
  } else {
    nodeHandle_.getParam("use_preview", usePreview);
  }
  return sendGoal(goal, usePreview);
}

void FreeGaitActionClient::sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal,
                                    const bool usePreview)
{
  if (usePreview && !goal.steps.empty()) {
    free_gait_msgs::ExecuteStepsActionGoal actionGoal;
    actionGoal.goal = goal;
    previewPublisher_.publish(actionGoal);
    actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::ACTIVE);
    free_gait_msgs::ExecuteStepsResult result;
    doneCallback_(state, result);
  } else {
    if (state_ == ActionState::ACTIVE || state_ == ActionState::PENDING) {
      client_->stopTrackingGoal();
    }
    if (!goal.steps.empty()) {
      setState(ActionState::PENDING);
    }
    client_->waitForServer();
    client_->sendGoal(goal, boost::bind(&FreeGaitActionClient::doneCallback, this, _1, _2),
                      boost::bind(&FreeGaitActionClient::activeCallback, this),
                      boost::bind(&FreeGaitActionClient::feedbackCallback, this, _1));
  }
}

void FreeGaitActionClient::cancelGoal() {
  return client_->cancelGoal();
}

bool FreeGaitActionClient::waitForResult(const double timeout)
{
  return client_->waitForResult(ros::Duration(timeout));
}

const FreeGaitActionClient::ActionState& FreeGaitActionClient::getState()
{
  return state_;
}

void FreeGaitActionClient::setState(const ActionState state)
{
  state_ = state;
  free_gait_msgs::ExecuteActionFeedback msg;
  msg.status = state_;
  statePublisher_.publish(msg);

}

void FreeGaitActionClient::activeCallback()
{
  setState(ActionState::ACTIVE);
  if (activeCallback_) {
    activeCallback_();
  }
}

void FreeGaitActionClient::feedbackCallback(
    const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback)
{
  if (feedbackCallback_) feedbackCallback_(feedback);
}

void FreeGaitActionClient::doneCallback(const actionlib::SimpleClientGoalState& state,
                                        const free_gait_msgs::ExecuteStepsResultConstPtr& result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    setState(ActionState::SUCCEEDED);
  } else if (state == actionlib::SimpleClientGoalState::PREEMPTED || state == actionlib::SimpleClientGoalState::RECALLED) {
    setState(ActionState::PREEMPTED);
  } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
    setState(ActionState::ERROR);
  } else {
    MELO_WARN_STREAM("Free Gait action client received goal state " << state.toString() << " upon action end. Action state will be considered as failed!");
    setState(ActionState::ERROR);
  }
  if (doneCallback_)
    doneCallback_(state, *result);
}

}
