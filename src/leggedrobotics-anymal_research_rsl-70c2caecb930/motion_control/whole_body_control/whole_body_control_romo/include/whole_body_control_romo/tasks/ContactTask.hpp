/*
 * ContactTask.hpp
 *
 *  Created on: July 18, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"
#include "whole_body_control_romo/tasks/TaskRomo.hpp"
#include "whole_body_control_romo/typedefs.hpp"
#include <basic_controllers/PIDGains.hpp>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class ContactTask : public TaskRomo<ConcreteDescription_, RobotState_> {
 protected:
  using PIDGainsD = basic_controllers::PIDGains3D;
  using Base = TaskRomo<ConcreteDescription_, RobotState_>;
  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = romo::RobotDescription<ConcreteDescription_>;

  bool adaptSingularValues_;
  double singularValueThreshold_;

  //! Gains for end-effector damping.
  PIDGainsD pidGains_;

 public:
  explicit ContactTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState), adaptSingularValues_(false), singularValueThreshold_(0.0) {
    this->inequalityConstraintJacobian_ = Eigen::MatrixXd();
    this->inequalityConstraintMaxValues_ = Eigen::VectorXd();
    this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd();
    this->constraintType_ = hopt::ConstraintType::Equality;
  }

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<ContactTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }

  static std::string getTaskTypeName() { return "contact"; }

  //! Load parameters
  bool loadParameters(TiXmlHandle taskHandle) override {
    if (!Base::loadParameters(taskHandle)) { return false; }

    TiXmlHandle contactTaskHandle = taskHandle;
    if (tinyxml_tools::getChildHandle(contactTaskHandle, taskHandle, "Contact", false)) {
      if (!tinyxml_tools::loadParameter(adaptSingularValues_, contactTaskHandle, "adapt_singular_values")) { return false; }
      tinyxml_tools::loadParameter(singularValueThreshold_, contactTaskHandle, "singular_value_threshold", 0.1);

      // Load d-gains.
      const std::vector<std::string> vectorParameterNames = {"x", "y", "z"};
      TiXmlHandle gainsHandle = contactTaskHandle;
      if (!tinyxml_tools::getChildHandle(gainsHandle, contactTaskHandle, "Gains")) {
        return false;
      }
      if (!tinyxml_tools::loadParameter("Derivative", pidGains_.derivativeGains_, gainsHandle.ToElement(), vectorParameterNames)) {
        return false;
      }

    } else {
      MELO_WARN_STREAM("[ContactTask::loadParameters] No \"Contact\" tag found in task "<<this->getName()<<"; all parameters are default.");
    }

    return true;
  }

  bool update(double dt, int solutionSpaceDimension) override {
    const auto& supportJacobian = this->getWholeBodyState().getSupportJacobian();
    const auto numContactConstraints = supportJacobian.getNumberOfTotalContactConstraints();
    const auto numContacts = supportJacobian.getNumberOfContacts();

    // End-Effector damping.
    const Eigen::VectorXd linearVelocityEndEffectorsInForceFrame = (supportJacobian.getSupportJacobianInForceFrame() *
                                   this->getWholeBodyState().getRobotModel().getState().getGeneralizedVelocities());

    Eigen::VectorXd referenceLinearAccelerationPointOnBodyInForceFrame = Eigen::VectorXd::Zero(numContactConstraints);

    auto legId = 0u;
    for (const auto contactKey : RD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      const auto branchEnum = RD::template mapEnums<typename RD::BranchEnum>(contactEnum);
      const auto limbEnum = RD::template mapEnums<typename RD::LimbEnum>(branchEnum);

      const auto& contactState = this->getWholeBodyState().getContactFlags()[limbEnum];
      if (contactState != ContactStateEnum::ContactOpen) {
        const auto contactDof = supportJacobian.getNumberOfContactConstraintsFromContactEnum(contactState);
        if (contactState == ContactStateEnum::ContactClosed3Dof) {
          referenceLinearAccelerationPointOnBodyInForceFrame.segment(legId, contactDof) =
              -pidGains_.derivativeGains_.getValue().cwiseProduct(linearVelocityEndEffectorsInForceFrame.segment(legId, contactDof));
        } else {
          MELO_WARN_THROTTLE(5.0, "Cannot set reference linear acceleration for 6 DoF contacts!");
        }
        legId += contactDof;
      }
    }

    this->equalityConstraintRelativeWeights_ = Eigen::VectorXd::Ones(numContactConstraints);
    this->equalityConstraintTargetValues_ = referenceLinearAccelerationPointOnBodyInForceFrame -
        (supportJacobian.getSupportJacobianTimeDerivativeInForceFrame() * this->getWholeBodyState().getRobotModel().getState().getGeneralizedVelocities());
    this->equalityConstraintJacobian_.resize(numContactConstraints, solutionSpaceDimension);

    if (adaptSingularValues_) {
      Eigen::MatrixXd dampedSupportJacobian = Eigen::MatrixXd(numContactConstraints, solutionSpaceDimension);
      dampedSupportJacobian << supportJacobian.getSupportJacobianInForceFrame(),
                               Eigen::MatrixXd::Zero(numContactConstraints, numContactConstraints);

      // Damp parts.
      auto counter = 0u;
      for (const auto contactKey : RD::getContactKeys()) {
        const auto contactEnum = contactKey.getEnum();
        const auto branchEnum = RD::template mapEnums<typename RD::BranchEnum>(contactEnum);
        const auto limbEnum = RD::template mapEnums<typename RD::LimbEnum>(branchEnum);

        const auto& contactState = this->getWholeBodyState().getContactFlags()[limbEnum];
        if (contactState != ContactStateEnum::ContactOpen) {
          const auto contactDof = supportJacobian.getNumberOfContactConstraintsFromContactEnum(contactState);
          const auto firstColumnIndex = RD::getBranchStartIndexInU(branchEnum);

          Eigen::JacobiSVD<Eigen::MatrixXd> Asvd(dampedSupportJacobian.block(counter,firstColumnIndex, contactDof,contactDof),
                                                 Eigen::ComputeThinU | Eigen::ComputeThinV);
          Eigen::VectorXd singularValues = Asvd.singularValues();

          const double minimumSingularValue = singularValues(Asvd.nonzeroSingularValues()-1);
          if (minimumSingularValue < this->singularValueThreshold_) {
            singularValues(Asvd.nonzeroSingularValues()-1) = this->singularValueThreshold_;
          }
          dampedSupportJacobian.block(counter,firstColumnIndex,contactDof,contactDof) = Asvd.matrixU()*singularValues.asDiagonal()*Asvd.matrixV().transpose();

          // Update the counter for the next iteration.
          counter += contactDof;
        }
      }

      this->equalityConstraintJacobian_ = dampedSupportJacobian;
    } else {
      this->equalityConstraintJacobian_.template leftCols<RD::getNumDof()>() = supportJacobian.getSupportJacobianInForceFrame();
      this->equalityConstraintJacobian_.rightCols(numContactConstraints).setZero();
    }

    return true;
  }
};

}  // namespace whole_body_control_romo
