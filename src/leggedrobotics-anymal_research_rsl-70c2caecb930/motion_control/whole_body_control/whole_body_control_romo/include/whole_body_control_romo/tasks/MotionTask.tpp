/*
 * MotionTask.tpp
 *
 *  Created on: May 7, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole body control
#include "whole_body_control_romo/tasks/MotionTask.hpp"

// tinyxml
#include <tinyxml_tools/tinyxml_tools.hpp>

// parameter handler
#include <parameter_handler/parameter_handler.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
bool MotionTask<ConcreteDescription_, RobotState_>::loadParameters(TiXmlHandle taskHandle) {
  TiXmlHandle motionTaskHandle = taskHandle;
  if (!tinyxml_tools::getChildHandle(motionTaskHandle, taskHandle, "Motion")) {
    return false;
  }

  std::string motionTaskFrameString;
  if (!tinyxml_tools::loadParameter(motionTaskFrameString, motionTaskHandle.ToElement(), "frame")) {
    return false;
  }
  try {
    motionTaskFrameEnum_ = motionTaskFrameKeys_.atName(motionTaskFrameString).getEnum();
  } catch (std::out_of_range& e) {
    MELO_WARN_STREAM("Motion task frame " << motionTaskFrameString << " does not exist." << e.what());
    return false;
  }

  // Load pid gains
  const std::vector<std::string> vectorParameterNames = {"x", "y", "z"};
  TiXmlHandle gainsHandle = motionTaskHandle;
  if (!tinyxml_tools::getChildHandle(gainsHandle, motionTaskHandle, "Gains")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter("Proportional", gainsInTaskFrame_.proportionalGains_, gainsHandle.ToElement(), vectorParameterNames)) {
    return false;
  }
  if (!tinyxml_tools::loadParameter("Integral", gainsInTaskFrame_.integralGains_, gainsHandle.ToElement(), vectorParameterNames)) {
    return false;
  }
  if (!tinyxml_tools::loadParameter("Derivative", gainsInTaskFrame_.derivativeGains_, gainsHandle.ToElement(), vectorParameterNames)) {
    return false;
  }
  if (!tinyxml_tools::loadParameter("MaxIntegral", gainsInTaskFrame_.maxIntegral_, gainsHandle.ToElement(), vectorParameterNames)) {
    return false;
  }

  // Load relative weights
  if (!tinyxml_tools::loadParameter("Weights", relativeWeightsInTaskFrame_, motionTaskHandle.ToElement(), vectorParameterNames)) {
    return false;
  }

  return TaskRomo<ConcreteDescription_, RobotState_>::loadParameters(taskHandle);
}

template <typename ConcreteDescription_, typename RobotState_>
bool MotionTask<ConcreteDescription_, RobotState_>::addParametersToHandler() {
  auto getParamName = [this](const std::string& s) {
    return std::string{"Wbc: "} + this->getName() + std::string{" "} + s + std::string{"("} +
           std::string{motionTaskFrameKeys_.at(motionTaskFrameEnum_).getName()} + std::string{" frame)"};
  };

  parameter_handler::handler->addParam(getParamName("Kp"), gainsInTaskFrame_.proportionalGains_);
  parameter_handler::handler->addParam(getParamName("Ki"), gainsInTaskFrame_.integralGains_);
  parameter_handler::handler->addParam(getParamName("Kd"), gainsInTaskFrame_.derivativeGains_);
  parameter_handler::handler->addParam(getParamName("MaxI"), gainsInTaskFrame_.maxIntegral_);
  parameter_handler::handler->addParam(getParamName("RelWeights"), relativeWeightsInTaskFrame_);

  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
bool MotionTask<ConcreteDescription_, RobotState_>::addVariablesToLog(const std::string& ns) const {
  const std::string& taskName = this->name_;
  const std::string prefix{ns + "/whole_body_control/" + taskName + "/"};
  if (taskName.find("translation") != std::string::npos) {
    signal_logger::add(linearPositionErrorInTaskFrame_, std::string{"positionErrorInTaskFrame"}, prefix, "m");
    signal_logger::add(linearVelocityErrorInTaskFrame_, std::string{"linearVelocityErrorInTaskFrame"}, prefix, "m/s");
  } else if (taskName.find("rotation") != std::string::npos) {
    signal_logger::add(angularPositionErrorInTaskFrame_, std::string{"orientationErrorInTaskFrame"}, prefix, "rad/s");
    signal_logger::add(angularVelocityErrorInTaskFrame_, std::string{"angularVelocityErrorInTaskFrame"}, prefix, "-");
  }
  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
bool MotionTask<ConcreteDescription_, RobotState_>::initialize(double dt) {
  linearPositionErrorInTaskFrame_.setZero();
  angularPositionErrorInTaskFrame_.setZero();
  linearVelocityErrorInTaskFrame_.setZero();
  angularVelocityErrorInTaskFrame_.setZero();
  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
bool MotionTask<ConcreteDescription_, RobotState_>::setOptimizationTaskPositionTrackingInSourceFrame(
    int solutionSpaceDimension, MotionTaskFrame sourceFrame, Vector3dConstRef desiredPositionWorldToPointOnBodyInSourceFrame,
    Vector3dConstRef desiredLinearVelocityPointOnBodyInSourceFrame, Vector3dConstRef desiredLinearAccelerationPointOnBodyInSourceFrame,
    Vector3dConstRef measuredPositionWorldToPointOnBodyInSourceFrame, Vector3dConstRef measuredLinearVelocityPointOnBodyInSourceFrame,
    JacobianTranslationConstRef jacobianTranslationWorldToPointOnBodyInSourceFrame,
    JacobianTranslationConstRef jacobianTranslationTimeDerivativeWorldToPointOnBodyInSourceFrame, bool appendToCurrentTask) {
  // Get orientation from source to task.
  const RotationMatrix orientationSourceFrameToTaskFrame(getOrientationSourceFrameToTaskFrame(sourceFrame));

  // Create matrices with number of non-zero weight constraints.
  const Eigen::Vector3d relativeWeightsInTaskFrame = relativeWeightsInTaskFrame_.getValue();
  const auto nDim = (relativeWeightsInTaskFrame.array() != 0.0).count();

  // Reset.
  const auto offset = appendToCurrentTask ? this->equalityConstraintTargetValues_.rows() : 0;
  const auto nDimStacked = offset + nDim;
  this->constraintType_ = hopt::ConstraintType::Equality;
  this->equalityConstraintJacobian_.conservativeResize(nDimStacked, solutionSpaceDimension);
  this->equalityConstraintJacobian_.bottomRows(nDim).setZero();
  this->equalityConstraintTargetValues_.conservativeResize(nDimStacked);
  this->equalityConstraintRelativeWeights_.conservativeResize(nDimStacked);

  // Compute motion tracking errors.
  linearPositionErrorInTaskFrame_ = orientationSourceFrameToTaskFrame.matrix() *
                                    (desiredPositionWorldToPointOnBodyInSourceFrame - measuredPositionWorldToPointOnBodyInSourceFrame);
  linearVelocityErrorInTaskFrame_ = orientationSourceFrameToTaskFrame.matrix() *
                                    (desiredLinearVelocityPointOnBodyInSourceFrame - measuredLinearVelocityPointOnBodyInSourceFrame);

  // Compute reference acceleration.
  const Eigen::Vector3d referenceLinearAccelerationPointOnBodyInTaskFrame =
      orientationSourceFrameToTaskFrame.matrix() * (desiredLinearAccelerationPointOnBodyInSourceFrame) +
      gainsInTaskFrame_.proportionalGains_.getValue().cwiseProduct(linearPositionErrorInTaskFrame_) +
      gainsInTaskFrame_.derivativeGains_.getValue().cwiseProduct(linearVelocityErrorInTaskFrame_);

  JacobianTranslationConstRef jacobianTranslationWorldToPointOnBodyInTaskFrame =
      orientationSourceFrameToTaskFrame.matrix() * jacobianTranslationWorldToPointOnBodyInSourceFrame;
  JacobianTranslationConstRef jacobianTranslationTimeDerivativeWorldToPointOnBodyInTaskFrame =
      orientationSourceFrameToTaskFrame.matrix() * jacobianTranslationTimeDerivativeWorldToPointOnBodyInSourceFrame;

  for (int i = 0, k = 0; i < 3; ++i) {
    // Build up the task optimization.
    if (relativeWeightsInTaskFrame(i) != 0.0) {
      this->equalityConstraintJacobian_.template block<1, RD::getNumDof()>(offset + k, 0) =
          jacobianTranslationWorldToPointOnBodyInTaskFrame.row(i);
      this->equalityConstraintTargetValues_(offset + k) =
          referenceLinearAccelerationPointOnBodyInTaskFrame(i) -
          jacobianTranslationTimeDerivativeWorldToPointOnBodyInTaskFrame.row(i) *
              this->getWholeBodyState().getRobotModel().getState().getGeneralizedVelocities();
      this->equalityConstraintRelativeWeights_(offset + k) = relativeWeightsInTaskFrame(i);
      ++k;
    }
  }

  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
bool MotionTask<ConcreteDescription_, RobotState_>::setOptimizationTaskOrientationTrackingInSourceFrame(
    int solutionSpaceDimension, MotionTaskFrame sourceFrame, const RotationQuaternion& desiredOrientationSourceFrameToPointOnBody,
    Vector3dConstRef desiredAngularVelocityPointOnBodyInSourceFrame, Vector3dConstRef desiredAngularAccelerationPointOnBodyInSourceFrame,
    const RotationQuaternion& measuredOrientationSourceFrameToPointOnBody, Vector3dConstRef measuredAngularVelocityPointOnBodyInSourceFrame,
    JacobianTranslationConstRef jacobianRotationWorldToPointOnBodyInSourceFrame,
    JacobianTranslationConstRef jacobianRotationTimeDerivativeWorldToPointOnBodyInSourceFrame, bool appendToCurrentTask) {
  // Get orientation from source to task.
  const RotationMatrix orientationSourceFrameToTaskFrame(getOrientationSourceFrameToTaskFrame(sourceFrame));

  // Create matrices with number of non-zero weight constraints.
  const Eigen::Vector3d& relativeWeightsInTaskFrame = relativeWeightsInTaskFrame_.getValue();
  const auto nDim = (relativeWeightsInTaskFrame.array() != 0.0).count();

  // Reset.
  const auto offset = appendToCurrentTask ? this->equalityConstraintTargetValues_.rows() : 0;
  const auto nDimStacked = offset + nDim;
  this->constraintType_ = hopt::ConstraintType::Equality;
  this->equalityConstraintJacobian_.conservativeResize(nDimStacked, solutionSpaceDimension);
  this->equalityConstraintJacobian_.bottomRows(nDim).setZero();
  this->equalityConstraintTargetValues_.conservativeResize(nDimStacked);
  this->equalityConstraintRelativeWeights_.conservativeResize(nDimStacked);

  // Compute motion tracking errors.
  angularPositionErrorInTaskFrame_ =
      orientationSourceFrameToTaskFrame.matrix() *
      (desiredOrientationSourceFrameToPointOnBody.inverted().boxMinus(measuredOrientationSourceFrameToPointOnBody.inverted()));
  angularVelocityErrorInTaskFrame_ = orientationSourceFrameToTaskFrame.matrix() *
                                     (desiredAngularVelocityPointOnBodyInSourceFrame - measuredAngularVelocityPointOnBodyInSourceFrame);

  // Compute reference acceleration.
  const Eigen::Vector3d referenceAngularAccelerationPointOnBodyInTaskFrame =
      orientationSourceFrameToTaskFrame.matrix() * (desiredAngularAccelerationPointOnBodyInSourceFrame) +
      gainsInTaskFrame_.proportionalGains_.getValue().cwiseProduct(angularPositionErrorInTaskFrame_) +
      gainsInTaskFrame_.derivativeGains_.getValue().cwiseProduct(angularVelocityErrorInTaskFrame_);

  JacobianTranslationConstRef jacobianRotationWorldToPointOnBodyInTaskFrame =
      orientationSourceFrameToTaskFrame.matrix() * jacobianRotationWorldToPointOnBodyInSourceFrame;
  JacobianTranslationConstRef jacobianRotationTimeDerivativeWorldToPointOnBodyInTaskFrame =
      orientationSourceFrameToTaskFrame.matrix() * jacobianRotationTimeDerivativeWorldToPointOnBodyInSourceFrame;

  // Build up the task optimization.
  for (int i = 0, k = 0; i < 3; ++i) {
    if (relativeWeightsInTaskFrame(i) != 0.0) {
      this->equalityConstraintJacobian_.template block<1, RD::getNumDof()>(offset + k, 0) =
          jacobianRotationWorldToPointOnBodyInTaskFrame.row(i);
      this->equalityConstraintTargetValues_(offset + k) =
          referenceAngularAccelerationPointOnBodyInTaskFrame(i) -
          jacobianRotationTimeDerivativeWorldToPointOnBodyInTaskFrame.row(i) *
              this->getWholeBodyState().getRobotModel().getState().getGeneralizedVelocities();
      this->equalityConstraintRelativeWeights_(offset + k) = relativeWeightsInTaskFrame(i);
      ++k;
    }
  }
  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
RotationQuaternion MotionTask<ConcreteDescription_, RobotState_>::getOrientationFrameAToFrameB(MotionTaskFrame frameA,
                                                                                               MotionTaskFrame frameB) const {
  // Switch depending on frame of task
  const auto& torsoStateMeasured = this->getWholeBodyState().getWholeBody().getTorso().getMeasuredState();

  switch (frameB) {
    case MotionTaskFrame::WORLD: {
      switch (frameA) {
        case MotionTaskFrame::CONTROL:
          return torsoStateMeasured.inControlFrame().getOrientationWorldToControl().inverted();
        case MotionTaskFrame::BASE:
          return torsoStateMeasured.getOrientationWorldToBase().inverted();
        default:
          return RotationQuaternion();
      }
    } break;
    case MotionTaskFrame::CONTROL: {
      switch (frameA) {
        case MotionTaskFrame::WORLD:
          return torsoStateMeasured.inControlFrame().getOrientationWorldToControl();
        case MotionTaskFrame::BASE:
          return torsoStateMeasured.inControlFrame().getOrientationControlToBase().inverted();
        default:
          return RotationQuaternion();
      }
    } break;
    case MotionTaskFrame::BASE: {
      switch (frameA) {
        case MotionTaskFrame::WORLD:
          return torsoStateMeasured.getOrientationWorldToBase();
        case MotionTaskFrame::CONTROL:
          return torsoStateMeasured.inControlFrame().getOrientationControlToBase();
        default:
          return RotationQuaternion();
      }
    } break;
    default:
      break;
  }

  return RotationQuaternion();
}

template <typename ConcreteDescription_, typename RobotState_>
RotationQuaternion MotionTask<ConcreteDescription_, RobotState_>::getOrientationSourceFrameToTaskFrame(MotionTaskFrame sourceFrame) const {
  return getOrientationFrameAToFrameB(sourceFrame, motionTaskFrameEnum_);
}

}  // namespace whole_body_control_romo
