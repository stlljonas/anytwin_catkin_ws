/*
 * BaseMotionTask.hpp
 *
 *  Created on: May 11, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

#include "whole_body_control_romo/tasks/MotionTask.hpp"
#include "whole_body_control_romo/typedefs.hpp"

#include <tinyxml_tools/tinyxml_tools.hpp>

#include <type_traits>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class BaseMotionTask : public MotionTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = MotionTask<ConcreteDescription_, RobotState_>;
  using RD = typename Base::RD;
  using WholeBodyState = typename Base::WholeBodyState;

 public:
  BaseMotionTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {}

  bool addVariablesToLog(const std::string& ns) const override { return true; }
};

template <typename ConcreteDescription_, typename RobotState_>
class BaseRotationTask : public BaseMotionTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = BaseMotionTask<ConcreteDescription_, RobotState_>;
  using RD = typename Base::RD;
  using WholeBodyState = typename Base::WholeBodyState;

 public:
  BaseRotationTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {
    this->inequalityConstraintJacobian_ = Eigen::MatrixXd();
    this->inequalityConstraintMaxValues_ = Eigen::VectorXd();
    this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd();
    this->constraintType_ = hopt::ConstraintType::Equality;
  }

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<BaseRotationTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "base_rotation"; }

  bool update(double dt, int solutionSpaceDimension) override {
    const Eigen::Vector3d& relativeWeightsInTaskFrame = this->getRelativeWeightsInMotionTaskFrame();
    const auto nDim = (relativeWeightsInTaskFrame.array() != 0.0).count();

    this->equalityConstraintJacobian_.resize(nDim, solutionSpaceDimension);
    this->equalityConstraintJacobian_.setZero();
    this->equalityConstraintTargetValues_.resize(nDim);
    this->equalityConstraintRelativeWeights_.resize(nDim);

    const auto& torso = this->getWholeBodyState().getWholeBody().getTorso();
    const auto& baseState = this->getWholeBodyState().getBaseTaskState();
    const Eigen::Vector3d targetValues =
        (torso.getDesiredState().getAngularAccelerationTargetInControlFrame().toImplementation() +
         this->getGainsInMotionTaskFrame().proportionalGains_.getValue().cwiseProduct(baseState.getOrientationErrorInControlFrame()) +
         this->getGainsInMotionTaskFrame().derivativeGains_.getValue().cwiseProduct(
             baseState.getAngularVelocityErrorInControlFrame().toImplementation()) -
         baseState.getJacobianRotationDerivativeWorldToTargetInControlFrame() *
             this->getWholeBodyState().getRobotModel().getState().getGeneralizedVelocities());

    for (int i = 0, k = 0; i < 3; ++i) {
      // Build up the task optimization.
      if (relativeWeightsInTaskFrame(i) != 0.0) {
        this->equalityConstraintRelativeWeights_(k) = relativeWeightsInTaskFrame(i);
        this->equalityConstraintJacobian_.row(k).leftCols(RD::getNumDof()) =
            baseState.getJacobianRotationWorldToTargetInControlFrame().row(i);
        this->equalityConstraintTargetValues_(k) = targetValues(i);
        ++k;
      }
    }

    return true;
  }
};

template <typename ConcreteDescription_, typename RobotState_>
class BaseTranslationTask : public BaseMotionTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = BaseMotionTask<ConcreteDescription_, RobotState_>;
  using RD = typename Base::RD;
  using WholeBodyState = typename Base::WholeBodyState;

 public:
  BaseTranslationTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {
    this->inequalityConstraintJacobian_ = Eigen::MatrixXd();
    this->inequalityConstraintMaxValues_ = Eigen::VectorXd();
    this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd();
    this->constraintType_ = hopt::ConstraintType::Equality;
  }

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<BaseTranslationTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "base_translation"; }

  bool update(double dt, int solutionSpaceDimension) override {
    const Eigen::Vector3d& relativeWeightsInTaskFrame = this->getRelativeWeightsInMotionTaskFrame();
    const auto nDim = (relativeWeightsInTaskFrame.array() != 0.0).count();
    this->equalityConstraintJacobian_.resize(nDim, solutionSpaceDimension);
    this->equalityConstraintJacobian_.setZero();
    this->equalityConstraintTargetValues_.resize(nDim);
    this->equalityConstraintRelativeWeights_.resize(nDim);

    const auto& torso = this->getWholeBodyState().getWholeBody().getTorso();
    const auto& baseState = this->getWholeBodyState().getBaseTaskState();
    const Eigen::Vector3d targetValues = torso.getDesiredState().getLinearAccelerationTargetInControlFrame().toImplementation() +
                                         this->getGainsInMotionTaskFrame().proportionalGains_.getValue().cwiseProduct(
                                             baseState.getPositionErrorInControlFrame().toImplementation()) +
                                         this->getGainsInMotionTaskFrame().derivativeGains_.getValue().cwiseProduct(
                                             baseState.getLinearVelocityErrorInControlFrame().toImplementation()) -
                                         baseState.getJacobianTranslationDerivativeWorldToTargetInControlFrame() *
                                             this->getWholeBodyState().getRobotModel().getState().getGeneralizedVelocities();

    for (int i = 0, k = 0; i < 3; ++i) {
      // Build up the task optimization.
      if (relativeWeightsInTaskFrame(i) != 0.0) {
        this->equalityConstraintRelativeWeights_(k) = relativeWeightsInTaskFrame(i);
        this->equalityConstraintJacobian_.row(k).leftCols(RD::getNumDof()) =
            baseState.getJacobianTranslationWorldToTargetInControlFrame().row(i);
        this->equalityConstraintTargetValues_(k) = targetValues(i);
        ++k;
      }
    }

    return true;
  }
};
}  // namespace whole_body_control_romo
