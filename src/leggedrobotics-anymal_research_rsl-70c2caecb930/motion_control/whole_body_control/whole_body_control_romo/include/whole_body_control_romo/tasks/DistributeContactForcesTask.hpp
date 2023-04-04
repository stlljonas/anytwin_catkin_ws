/*
 * DistributeContactForcesTask.hpp
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
class DistributeContactForcesTask : public TaskRomo<ConcreteDescription_, RobotState_> {
 protected:
  using Base = TaskRomo<ConcreteDescription_, RobotState_>;
  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = romo::RobotDescription<ConcreteDescription_>;

 public:
  explicit DistributeContactForcesTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {
    this->inequalityConstraintJacobian_ = Eigen::MatrixXd();
    this->inequalityConstraintMaxValues_ = Eigen::VectorXd();
    this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd();
    this->constraintType_ = hopt::ConstraintType::Equality;
  }

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<DistributeContactForcesTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "distribute_contact_forces"; }

  bool update(double dt, int solutionSpaceDimension) override {
    const auto contactForcesDimenstion = solutionSpaceDimension - RD::getNumDof();
    this->equalityConstraintJacobian_ = Eigen::MatrixXd::Zero(contactForcesDimenstion, solutionSpaceDimension);
    this->equalityConstraintJacobian_.rightCols(contactForcesDimenstion).setIdentity();
    this->equalityConstraintTargetValues_ = Eigen::VectorXd::Zero(contactForcesDimenstion);
    this->equalityConstraintRelativeWeights_ = Eigen::VectorXd::Ones(contactForcesDimenstion);
    return true;
  }
};

}  // namespace whole_body_control_romo
