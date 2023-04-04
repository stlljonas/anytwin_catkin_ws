/*
 * JointVelocityTask.hpp
 *
 *  Created on: May 14, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"
#include "whole_body_control_romo/tasks/JointTask.hpp"
#include "whole_body_control_romo/typedefs.hpp"

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class JointAccelerationTask : public JointTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = JointTask<ConcreteDescription_, RobotState_>;
  using WholeBodyState = typename Base::WholeBodyState;
  using RD = typename Base::RD;

 public:
  explicit JointAccelerationTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {
    this->inequalityConstraintJacobian_ = Eigen::MatrixXd();
    this->inequalityConstraintMaxValues_ = Eigen::VectorXd();
    this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd();
    this->constraintType_ = hopt::ConstraintType::Equality;
  }

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<JointAccelerationTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "joint_acceleration"; }

  //! Load parameters
  bool loadParameters(TiXmlHandle taskHandle) override {
    // Get joint task handle
    TiXmlHandle jointTaskHandle = taskHandle;
    if (!tinyxml_tools::getChildHandle(jointTaskHandle, taskHandle, "Joint")) {
      return false;
    }

    // Get gain handle
    TiXmlHandle targetHandle = taskHandle;
    if (!tinyxml_tools::getChildHandle(targetHandle, jointTaskHandle, "Target")) {
      return false;
    }

    // Read gains
    if (!tinyxml_tools::loadParameter(targetValue_, targetHandle.ToElement(), "acceleration", 0.0)) {
      return false;
    }

    return Base::loadParameters(taskHandle);
  }

  bool update(double dt, int solutionSpaceDimension) override {
    const auto nDim = this->jointTaskInfos_.size();
    // [I 0] x	=  [qdd_des]
    this->equalityConstraintJacobian_ = Eigen::MatrixXd::Zero(nDim, solutionSpaceDimension);
    this->equalityConstraintTargetValues_ = Eigen::VectorXd::Zero(nDim);
    this->equalityConstraintRelativeWeights_ = Eigen::VectorXd::Ones(nDim);

    for (unsigned int k = 0; k < nDim; ++k) {
      const auto idInJ = this->jointTaskInfos_[k].idInLimbJ;
      const int offset = RD::getBaseGeneralizedVelocitiesDimension() + RD::getLimbStartIndexInJ(this->jointTaskInfos_[k].limbEnum) + idInJ;
      this->equalityConstraintJacobian_(k, offset) = 1;
      this->equalityConstraintTargetValues_(k) = targetValue_;
    }

    return true;
  }
 protected:
  double targetValue_;
};

}  // namespace whole_body_control_romo
