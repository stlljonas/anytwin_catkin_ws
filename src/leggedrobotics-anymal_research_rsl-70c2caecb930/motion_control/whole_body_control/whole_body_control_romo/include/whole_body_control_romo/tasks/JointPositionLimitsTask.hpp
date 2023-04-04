/*
 * JointPositionLimitsTask.hpp
 *
 *  Created on: June 4, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"
#include "whole_body_control_romo/tasks/JointTask.hpp"
#include "whole_body_control_romo/typedefs.hpp"

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class JointPositionLimitsTask : public JointLimitsTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = JointLimitsTask<ConcreteDescription_, RobotState_>;
  using WholeBodyState = typename Base::WholeBodyState;
  using RD = typename Base::RD;

 public:
  explicit JointPositionLimitsTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {
    this->equalityConstraintJacobian_ = Eigen::MatrixXd();
    this->equalityConstraintTargetValues_ = Eigen::VectorXd();
    this->equalityConstraintRelativeWeights_ = Eigen::VectorXd();
    this->constraintType_ = hopt::ConstraintType::Inequality;
  }

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<JointPositionLimitsTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "joint_position_limits"; }

  bool update(double dt, int solutionSpaceDimension) override {
    const double kt = dt * this->timestepScaling_;
    const int nConstraints = 2;
    const auto nDim = nConstraints * this->jointTaskInfos_.size();

    this->inequalityConstraintJacobian_ = Eigen::MatrixXd::Zero(nDim, solutionSpaceDimension);
    this->inequalityConstraintMaxValues_ = Eigen::VectorXd::Zero(nDim);
    this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd::Ones(nDim);

    for (unsigned int k = 0; k < this->jointTaskInfos_.size(); ++k) {
      const auto idInJ = this->jointTaskInfos_[k].idInLimbJ;
      const int offset = RD::getBaseGeneralizedVelocitiesDimension() +
          RD::getLimbStartIndexInJ(this->jointTaskInfos_[k].limbEnum) + idInJ;
      const auto& jointEnum = this->jointTaskInfos_[k].jointEnum;

      const auto limbStateMeasured = this->jointTaskInfos_[k].limb->getLimbStateMeasured();
      const auto unacceleratedPosition = limbStateMeasured.getJointPositions()(idInJ) + kt * limbStateMeasured.getJointVelocities()(idInJ);

      // Limit a * t^2 / 2 < x_max - x - v * t (from x + v * t + a * t^2 / 2 < x_max)
      this->inequalityConstraintJacobian_(nConstraints * k, offset) = 0.5 * kt * kt;
      this->inequalityConstraintMaxValues_(nConstraints * k) =
          this->model_.getLimits()->getJointMaxPosition(jointEnum) - unacceleratedPosition;

      // Limit -a * t^2 / 2 < - x_min + x + v * t (from x_min < x + v * t + a * t² / 2)
      this->inequalityConstraintJacobian_(nConstraints * k + 1, offset) = -0.5 * kt * kt;
      this->inequalityConstraintMaxValues_(nConstraints * k + 1) =
          - this->model_.getLimits()->getJointMinPosition(jointEnum) + unacceleratedPosition;
    }
    return true;
  }
};

}  // namespace whole_body_control_romo
