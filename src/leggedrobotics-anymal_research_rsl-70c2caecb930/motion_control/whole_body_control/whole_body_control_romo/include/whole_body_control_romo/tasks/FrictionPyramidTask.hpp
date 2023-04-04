/*
 * FrictionPyramidTask.hpp
 *
 *  Created on: May 22, 2018
 *      Author: Gabriel Hottiger
 *
 *  This task implements for all limbs in 3 DOF contact and specified in task xml
 *      a) tangential forces to be within the friction pyramid
 *      b) normal forces if this limb is listed in a tag <ForceLimit limbs="..." min="..." max="..." />
 *
 *  Once a limb loses contact, the contact force is not constrained anymore
 */

#pragma once

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"
#include "whole_body_control_romo/tasks/TaskRomo.hpp"
#include "whole_body_control_romo/typedefs.hpp"

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class FrictionPyramidTask : public TaskRomo<ConcreteDescription_, RobotState_> {
 protected:
  using Base = TaskRomo<ConcreteDescription_, RobotState_>;
  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = romo::RobotDescription<ConcreteDescription_>;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;
  using LimbEnum = typename RD::LimbEnum;

 public:
  explicit FrictionPyramidTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState), normalConstraintOnly_(false) {
    const auto getParameterName = [](const std::string& s) { return "Wbc: " + s + " normal force"; };

    // Create parameters for every limb
    for (auto limb : this->getWholeBodyState().getWholeBody().getLimbs()) {
      const auto limbEnum = RD::template mapKeyIdToKeyEnum<LimbEnum>(limb->getLimbUInt());
      contactForceLimits_[limbEnum] = {parameter_handler::Parameter<double>(getParameterName(limb->getName() + " min"),
                                                                            0, 0, std::numeric_limits<double>::max()),
                                       parameter_handler::Parameter<double>(getParameterName(limb->getName() + " max"),
                                                                            std::numeric_limits<double>::max(),
                                                                            0, std::numeric_limits<double>::max())};
    }

    this->equalityConstraintJacobian_ = Eigen::MatrixXd();
    this->equalityConstraintTargetValues_ = Eigen::VectorXd();
    this->equalityConstraintRelativeWeights_ = Eigen::VectorXd();
    this->constraintType_ = hopt::ConstraintType::Inequality;
  }

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<FrictionPyramidTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "friction_pyramid"; }

  //! Load parameters
  bool loadParameters(TiXmlHandle taskHandle) override {
    // Don't force handle 'NormalForceOnly', but if it's there require 'value' to be set.
    TiXmlHandle normalForceOnlyHandle = taskHandle;
    normalConstraintOnly_ = false;
    if (tinyxml_tools::getChildHandle(normalForceOnlyHandle, taskHandle, "NormalForceOnly", false)) {
      if (!tinyxml_tools::loadParameter(normalConstraintOnly_, normalForceOnlyHandle.ToElement(), "value", false)) {
        return false;
      }
    }

    // Get all definitions of force limits, later ones cannot overwrite previous declarations
    std::vector<TiXmlElement*> forceLimitElements;
    if (!tinyxml_tools::getChildElements(forceLimitElements, taskHandle, "ForceLimit")) {
      return false;
    }

    for (auto& elem : forceLimitElements) {
      // Get limbs from comma separated list.
      std::string limbList;
      if (!tinyxml_tools::loadParameter(limbList, elem, "limbs")) {
        return false;
      }
      double minForce;
      if (!tinyxml_tools::loadParameter(minForce, elem, "min")) {
        return false;
      }
      double maxForce;
      if (!tinyxml_tools::loadParameter(maxForce, elem, "max")) {
        return false;
      }

      std::vector<std::string> limbNames = this->getLimbNamesFromCommaSeperatedString(limbList);

      // Extend vector of specified limbs for this task and set force limits
      if (!setForceLimitsForTaskLimbs(limbNames, minForce, maxForce)) {
        return false;
      }
    }

    return Base::loadParameters(taskHandle);
  }
  bool initialize(double dt) override {
    /* One limb cannot carry more than total weight
       Times 1.5 to account for model inaccuracies and control commands (e.g. VMC) */
    const double maxForce = this->getWholeBodyState().getWholeBody().getWholeBodyProperties().getTotalMass() *
                            this->getWholeBodyState().getRobotModel().getGravityAcceleration() * 1.5;
    
    for (auto limb : this->getWholeBodyState().getWholeBody().getLimbs()) {
      const auto limbEnum = RD::template mapKeyIdToKeyEnum<LimbEnum>(limb->getLimbUInt());
      contactForceLimits_[limbEnum].maxForce.setMinValue(0.0);
      contactForceLimits_[limbEnum].maxForce.setMaxValue(maxForce);
      contactForceLimits_[limbEnum].minForce.setMinValue(0.0);
      contactForceLimits_[limbEnum].minForce.setMaxValue(maxForce);
      // Trigger new min/max values.
      contactForceLimits_[limbEnum].maxForce.setValue(contactForceLimits_[limbEnum].maxForce.getValue());
      contactForceLimits_[limbEnum].minForce.setValue(contactForceLimits_[limbEnum].minForce.getValue());
    }

    return true;
  }

  bool update(double dt, int solutionSpaceDimension) override {

    // Compute numContacts
    int numContacts = 0;
    for (const auto& limb : taskLimbs_) {
      LimbEnum limbEnum;
      try {
        limbEnum = RD::template mapKeyIdToKeyEnum<LimbEnum>(limb->getLimbUInt());
      } catch (std::out_of_range& e) {
        MELO_WARN_STREAM("[FrictionPyramidTask] Cannot map LimbId of limb " << limb->getName() << " to LimbEnum." << e.what());
        return false;
      }
      if (this->getWholeBodyState().getContactFlags()[limbEnum] == ContactStateEnum::ContactClosed3Dof) {
        ++numContacts;
      }
    }

    // Setup inequalities
    const int nIeqPerContact = normalConstraintOnly_ ? 2 : 6;
    this->inequalityConstraintJacobian_ = Eigen::MatrixXd::Zero(nIeqPerContact * numContacts, solutionSpaceDimension);
    this->inequalityConstraintMaxValues_ = Eigen::VectorXd::Zero(nIeqPerContact * numContacts);
    this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd::Ones(nIeqPerContact * numContacts);

    if (numContacts == 0) return true;

    // Get the orientation from the control frame to the frame in which the forces are defined.
    const auto& orientationControlToBase = this->getWholeBodyState()
                                               .getWholeBody()
                                               .getTorso()
                                               .getMeasuredState()
                                               .inControlFrame()
                                               .getOrientationControlToBase();
    const auto& orientationWorldToBase =
        this->getWholeBodyState().getWholeBody().getTorso().getMeasuredState().getOrientationWorldToBase();
    const loco::RotationQuaternion& orientationControlToForceFrame =
        (this->getWholeBodyState().getSupportJacobian().getForceFrame() == CoordinateFrameEnum::WORLD)
            ? orientationWorldToBase.inverted() * orientationControlToBase
            : orientationControlToBase;

    const Eigen::MatrixXd headingDirectionInForceFrameTranspose =
        orientationControlToForceFrame.rotate(loco::Vector::UnitX()).toImplementation().transpose();
    const Eigen::MatrixXd lateralDirectionInForceFrameTranspose =
        orientationControlToForceFrame.rotate(loco::Vector::UnitY()).toImplementation().transpose();
    const Eigen::MatrixXd normalDirectionInForceFrameTranspose =
        orientationControlToForceFrame.rotate(loco::Vector::UnitZ()).toImplementation().transpose();

    double mu = 0.0;
    this->getWholeBodyState().getTerrain().getFrictionCoefficientForFoot(loco::Position(), mu);
    unsigned int k = 0;

    for (const auto& limb : taskLimbs_) {
      LimbEnum limbEnum;
      try {
        limbEnum = RD::template mapKeyIdToKeyEnum<LimbEnum>(limb->getLimbUInt());
      } catch (std::out_of_range& e) {
        MELO_WARN_STREAM("[FrictionPyramidTask] Cannot map LimbId of limb " << limb->getName() << " to LimbEnum." << e.what());
        return false;
      }

      // Skip this limb if it is not in contact.
      if (this->getWholeBodyState().getContactFlags()[limbEnum] != ContactStateEnum::ContactClosed3Dof) {
        continue;
      }

      // Calculate the modulated friction.
      const double limbLoad = limb->getLoadFactor();
      //    if (limb->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::Support) {
      //      leg->setFrictionModulation(limbLoad);
      //    }
      const double modulatedFriction = mu * limb->getFrictionModulation();

      /** Friction cone constraint:
       *    Contact Force: Fc (part of solution vector)
       *    Normal force: Fn / Terrain normal: n
       *    Heading tangential: t_h / Lateral tangential: t_l
       *    Eq0:    -Fn_min > -n * Fc (from: Fn_min < n * Fc)
       *    Eq1:    Fn_max > n * Fc
       *    Eq2:    0 > (t_h - mu * n) * F_c (from: mu * n * F_c > t_h * F_c)
       *    Eq3:    0 > (t_l - mu * n) * F_c (from: mu * n * F_c > t_l * F_c)
       *    Eq4:    0 > -(t_h + mu * n) * F_c (from: mu * n * F_c > - t_h * F_c)
       *    Eq5:    0 > -(t_l + mu * n) * F_c (from: mu * n * F_c > - t_l * F_c)
       */
      const int indexInSupportJacobian = this->getWholeBodyState().getSupportJacobian().getStartIndexInSupportJacobian(limbEnum);

      this->inequalityConstraintJacobian_.template block<1, 3>(nIeqPerContact * k, RD::getNumDof() + indexInSupportJacobian) =
          -normalDirectionInForceFrameTranspose;
      this->inequalityConstraintJacobian_.template block<1, 3>(nIeqPerContact * k + 1, RD::getNumDof() + indexInSupportJacobian) =
          normalDirectionInForceFrameTranspose;
      if (!normalConstraintOnly_) {
        this->inequalityConstraintJacobian_.template block<1, 3>(nIeqPerContact * k + 2, RD::getNumDof() + indexInSupportJacobian) =
            (headingDirectionInForceFrameTranspose - modulatedFriction * normalDirectionInForceFrameTranspose);
        this->inequalityConstraintJacobian_.template block<1, 3>(nIeqPerContact * k + 3, RD::getNumDof() + indexInSupportJacobian) =
            (lateralDirectionInForceFrameTranspose - modulatedFriction * normalDirectionInForceFrameTranspose);
        this->inequalityConstraintJacobian_.template block<1, 3>(nIeqPerContact * k + 4, RD::getNumDof() + indexInSupportJacobian) =
            -(headingDirectionInForceFrameTranspose + modulatedFriction * normalDirectionInForceFrameTranspose);
        this->inequalityConstraintJacobian_.template block<1, 3>(nIeqPerContact * k + 5, RD::getNumDof() + indexInSupportJacobian) =
            -(lateralDirectionInForceFrameTranspose + modulatedFriction * normalDirectionInForceFrameTranspose);
      }
      this->inequalityConstraintMaxValues_(nIeqPerContact * k) = -contactForceLimits_.at(limbEnum).minForce.getValue();
      this->inequalityConstraintMaxValues_(nIeqPerContact * k + 1) =
          std::max(contactForceLimits_.at(limbEnum).minForce.getValue() + 0.01,
                   limbLoad * contactForceLimits_.at(limbEnum).maxForce.getValue());
      // Max values for Eq 3...5 are already initialized to 0

      ++k;
    }

    return true;
  }

  bool setForceLimitsForTaskLimbs(std::vector<std::string> limbNames, double minForce, double maxForce) {
    for (const auto& limbName : limbNames) {
      unsigned int limbId;
      // Get limb id
      try {
        limbId = static_cast<unsigned int>(RD::template mapKeyNameToKeyId<LimbEnum>(limbName));
      } catch (std::out_of_range &e) {
        MELO_WARN_STREAM("[FrictionPyramidTask]: Cannot map limbName " << limbName << " to LimbId." << e.what());
        return false;
      }
      // Get limb from id
      auto &limbs = this->getWholeBodyState().getWholeBody().getLimbs();
      auto it = std::find_if(limbs.begin(), limbs.end(),
                           [limbId](loco::LimbBase const *const limb) { return limb->getLimbUInt() == limbId; });
      if (it != limbs.end()) {
        // Check if taskLimbs_ does not yet contain that limb
        if ( std::find(taskLimbs_.begin(), taskLimbs_.end(), *it) != taskLimbs_.end() ) {
          MELO_WARN_STREAM("[FrictionPyramidTask]: Limb " << limbName
                           << " has already been specified as task limb. Will not overwrite. Fix task xml parameters.");
          continue;
        }
        else {
          // If limb exists and is not processed yet, add to taskLimbs_ and set force limits
          taskLimbs_.push_back(*it);

          LimbEnum limbEnum;
          try {
            limbEnum = RD::template mapKeyNameToKeyEnum<LimbEnum>(limbName);
          } catch (std::out_of_range& e) {
            MELO_WARN_STREAM("[FrictionPyramidTask] Cannot map limbName " << limbName << " to LimbEnum." << e.what());
            return false;
          }
          contactForceLimits_.at(limbEnum).minForce.setDefaultValue(minForce);
          contactForceLimits_.at(limbEnum).minForce.resetToDefault();
          contactForceLimits_.at(limbEnum).maxForce.setDefaultValue(maxForce);
          contactForceLimits_.at(limbEnum).maxForce.resetToDefault();

          MELO_DEBUG_STREAM("[FrictionPyramidTask] Limb "
                                   << limbName << " " << contactForceLimits_.at(limbEnum).minForce.getName()
                                   << " added force limits min: " << contactForceLimits_.at(limbEnum).minForce.getValue()
                                   << " max: " << contactForceLimits_.at(limbEnum).maxForce.getValue())
        }
      } else {
        MELO_ERROR_STREAM("[FrictionPyramidTask]: Limb " << limbName << " is not contained by wholeBody.");
        return false;
      }
    }
  return true;
  }

 private:
  struct ForceLimits {
    parameter_handler::Parameter<double> minForce;
    parameter_handler::Parameter<double> maxForce;
  };
  std_utils::EnumMap<LimbEnum, ForceLimits> contactForceLimits_;
  bool normalConstraintOnly_;
  std::vector<loco::LimbBase const*> taskLimbs_;
};

}  // namespace whole_body_control_romo
