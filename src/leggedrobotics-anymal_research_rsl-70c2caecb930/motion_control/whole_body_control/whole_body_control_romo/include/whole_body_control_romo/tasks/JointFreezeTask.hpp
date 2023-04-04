/*
 * JointFreezeTask.hpp
 *
 *  Created on: Sep 27, 2018
 *      Author: Simon Kerscher
 */

#pragma once


#include <loco/common/typedefs.hpp>
// whole_body_control_romo
#include "whole_body_control_romo/tasks/JointAccelerationTask.hpp"

namespace  whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class JointFreezeTask : public JointAccelerationTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = JointAccelerationTask<ConcreteDescription_, RobotState_>;
  using WholeBodyState = typename Base::WholeBodyState;
  using RD = typename Base::RD;

 public:
  explicit JointFreezeTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {
    this->equalityConstraintJacobian_ = Eigen::MatrixXd();
    this->equalityConstraintTargetValues_ = Eigen::VectorXd();
    this->equalityConstraintRelativeWeights_ = Eigen::VectorXd();
    this->constraintType_ = hopt::ConstraintType::Equality;
  }

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<JointFreezeTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "joint_freeze"; }

  // This update() method does not work with Base::setControlModeForLimbs(), as the base method will overwrite the control mode
  // Use a derived version if using this task
  bool update(double dt, int solutionSpaceDimension) override {
    for (auto&& jointTaskInfo : this->jointTaskInfos_) {
      auto limb = const_cast<loco::LimbBase*>(jointTaskInfo.limb);
      limb->getLimbStateDesiredPtr()->setJointControlMode(jointTaskInfo.idInLimbJ, loco::ControlMode::MODE_FREEZE);
    }

    return Base::update(dt, solutionSpaceDimension);
  }
};

} // namespace whole_body_control_romo