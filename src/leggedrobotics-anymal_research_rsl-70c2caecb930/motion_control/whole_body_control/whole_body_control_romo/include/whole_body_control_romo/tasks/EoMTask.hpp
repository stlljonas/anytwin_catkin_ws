/*
 * EomTask.hpp
 *
 *  Created on: May 15, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"
#include "whole_body_control_romo/tasks/TaskRomo.hpp"
#include "whole_body_control_romo/typedefs.hpp"

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class EoMTask : public TaskRomo<ConcreteDescription_, RobotState_> {
 protected:
  using Base = TaskRomo<ConcreteDescription_, RobotState_>;
  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = romo::RobotDescription<ConcreteDescription_>;

 public:
  explicit EoMTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {
    this->inequalityConstraintJacobian_ = Eigen::MatrixXd();
    this->inequalityConstraintMaxValues_ = Eigen::VectorXd();
    this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd();
    this->equalityConstraintRelativeWeights_ = Eigen::VectorXd::Ones(RD::getBaseGeneralizedVelocitiesDimension());
    this->constraintType_ = hopt::ConstraintType::Equality;
  }

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<EoMTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "eom"; }

  bool update(double dt, int solutionSpaceDimension) override {
    constexpr auto floatingBaseDimension = RD::getBaseGeneralizedVelocitiesDimension();
    this->equalityConstraintJacobian_ = Eigen::MatrixXd(floatingBaseDimension, solutionSpaceDimension);
    this->equalityConstraintJacobian_.template leftCols<RD::getNumDof()>() =
        this->getWholeBodyState().getWholeBody().getWholeBodyMassMatrix().template topRows<floatingBaseDimension>();
    this->equalityConstraintJacobian_.rightCols(solutionSpaceDimension - RD::getNumDof()) =
        (-this->getWholeBodyState().getSupportJacobian()
             .getSupportJacobianTransposeInForceFrame()
             .template topRows<floatingBaseDimension>());
    this->equalityConstraintTargetValues_ =
        (-this->getWholeBodyState().getWholeBody().getWholeBodyNonlinearEffects().template head<floatingBaseDimension>());
    return true;
  }
};

}  // namespace whole_body_control_romo
