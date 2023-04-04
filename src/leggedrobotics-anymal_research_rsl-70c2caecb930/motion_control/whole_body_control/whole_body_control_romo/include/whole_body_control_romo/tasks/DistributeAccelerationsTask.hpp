/*
 * DistributeAccelerationsTask.hpp
 *
 *  Created on: May 22, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"
#include "whole_body_control_romo/tasks/TaskRomo.hpp"
#include "whole_body_control_romo/typedefs.hpp"

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class DistributeAccelerationsTask : public TaskRomo<ConcreteDescription_, RobotState_> {
 protected:
  using Base = TaskRomo<ConcreteDescription_, RobotState_>;
  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = romo::RobotDescription<ConcreteDescription_>;

 public:
  explicit DistributeAccelerationsTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {
    this->inequalityConstraintJacobian_ = Eigen::MatrixXd();
    this->inequalityConstraintMaxValues_ = Eigen::VectorXd();
    this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd();
    this->equalityConstraintJacobian_ = Eigen::MatrixXd::Identity(RD::getNumDof(), RD::getNumDof());
    this->equalityConstraintTargetValues_ = Eigen::VectorXd::Zero(RD::getNumDof());
    this->equalityConstraintRelativeWeights_ = Eigen::VectorXd::Ones(RD::getNumDof());
    this->constraintType_ = hopt::ConstraintType::Equality;
  }

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<DistributeAccelerationsTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "distribute_accelerations"; }

  bool update(double dt, int solutionSpaceDimension) override {
    this->equalityConstraintJacobian_.conservativeResize(RD::getNumDof(), solutionSpaceDimension);
    this->equalityConstraintJacobian_.rightCols(solutionSpaceDimension - RD::getNumDof()).setZero();
    return true;
  }
};

}  // namespace whole_body_control_romo
