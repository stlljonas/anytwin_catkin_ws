/*
 * EndEffectorMotionTask.hpp
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
class EndEffectorMotionTask : public MotionTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = MotionTask<ConcreteDescription_, RobotState_>;
  using RD = typename Base::RD;
  using LimbEnum = typename RD::LimbEnum;

  using WholeBodyState = typename Base::WholeBodyState;

 public:
  EndEffectorMotionTask(WholeBodyState& wholeBodyState)
      : Base(wholeBodyState), limbs_(), endEffectorFrame_(loco::EndEffectorContactEnum::Origin) {}

  //! Load parameters
  bool loadParameters(TiXmlHandle taskHandle) override {
    TiXmlHandle motionTaskHandle = taskHandle;
    if (!tinyxml_tools::getChildHandle(motionTaskHandle, taskHandle, "Motion")) {
      return false;
    }

    // Get limbs from comma separated list
    std::string limbList;
    if (!tinyxml_tools::loadParameter(limbList, motionTaskHandle.ToElement(), "limbs")) {
      return false;
    }
    std::vector<std::string> limbNames = this->getLimbNamesFromCommaSeperatedString(limbList);
    limbs_.reserve(limbNames.size());

    for (const auto& limbName : limbNames) {
      unsigned int limbId;
      try {
        // Get branch
        limbId = static_cast<unsigned int>(RD::template mapKeyNameToKeyId<LimbEnum>(limbName));
      } catch (std::out_of_range& e) {
        MELO_WARN_STREAM("[EndEffectorMotionTask]: Limb " << limbName << " does not exist." << e.what());
        return false;
      }
      //! Get endeffectors from id
      auto& limbs = this->getWholeBodyState().getWholeBody().getLimbs();
      auto it =
          std::find_if(limbs.begin(), limbs.end(), [limbId](loco::LimbBase const* const limb) { return limb->getLimbUInt() == limbId; });
      if (it != limbs.end()) {
        limbs_.push_back(*it);
      } else {
        MELO_WARN_STREAM("[EndEffectorMotionTask]: Limb " << limbName << " does not exist.");
        return false;
      }
    }

    //! Get endeffector frame
    std::string endeffectorPointString;
    if (!tinyxml_tools::loadParameter(endeffectorPointString, motionTaskHandle.ToElement(), "point")) {
      return false;
    }
    if (endeffectorPointString == "origin") {
      endEffectorFrame_ = loco::EndEffectorContactEnum::Origin;
    } else if (endeffectorPointString == "contact") {
      endEffectorFrame_ = loco::EndEffectorContactEnum::Contact;
    } else {
      MELO_WARN_STREAM("[EndEffectorMotionTask]: Point " << endeffectorPointString << " does not exist.");
      return false;
    }

    return Base::loadParameters(taskHandle);
  }

 protected:
  std::vector<loco::LimbBase const*> limbs_;
  loco::EndEffectorBase::EndEffectorFrame endEffectorFrame_;
};

template <typename ConcreteDescription_, typename RobotState_>
class EndEffectorRotationTask : public EndEffectorMotionTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = EndEffectorMotionTask<ConcreteDescription_, RobotState_>;
  using RD = typename Base::RD;
  using WholeBodyState = typename Base::WholeBodyState;
  using LimbEnum = typename RD::LimbEnum;

 public:
  EndEffectorRotationTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {}

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<EndEffectorRotationTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "endeffector_rotation"; }

  bool update(double dt, int solutionSpaceDimension) override {
    bool appendToTask = false;

    this->equalityConstraintJacobian_ = Eigen::MatrixXd();
    this->equalityConstraintTargetValues_ = Eigen::VectorXd();
    this->equalityConstraintRelativeWeights_ = Eigen::VectorXd();

    for (auto limb : this->limbs_) {
      if (this->getWholeBodyState().getContactFlags()[RD::template mapKeyIdToKeyEnum<LimbEnum>(limb->getLimbUInt())] ==
          ContactStateEnum::ContactOpen) {
        const auto& desiredState = limb->getEndEffector().getStateDesired(loco::TimePoint::Now, this->endEffectorFrame_);
        const auto& measuredState = limb->getEndEffector().getStateMeasured(loco::TimePoint::Now, this->endEffectorFrame_);
        this->setOptimizationTaskOrientationTrackingInSourceFrame(
            solutionSpaceDimension, MotionTaskFrame::WORLD, desiredState.getOrientationWorldToEndEffector(),
            desiredState.getAngularVelocityEndEffectorInWorldFrame().toImplementation(),
            desiredState.getAngularAccelerationEndEffectorInWorldFrame().toImplementation(),
            measuredState.getOrientationWorldToEndEffector(), measuredState.getAngularVelocityEndEffectorInWorldFrame().toImplementation(),
            measuredState.getRotationJacobianWorldToEndEffectorInWorldFrame(),
            measuredState.getRotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame(), appendToTask);
        appendToTask = true;
      }
    }
    return true;
  }
};

template <typename ConcreteDescription_, typename RobotState_>
class EndEffectorTranslationTask : public EndEffectorMotionTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = EndEffectorMotionTask<ConcreteDescription_, RobotState_>;
  using RD = typename Base::RD;
  using WholeBodyState = typename Base::WholeBodyState;
  using LimbEnum = typename RD::LimbEnum;

 public:
  EndEffectorTranslationTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {}

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<EndEffectorTranslationTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "endeffector_translation"; }

  bool update(double dt, int solutionSpaceDimension) override {
    bool appendToTask = false;

    this->equalityConstraintJacobian_ = Eigen::MatrixXd();
    this->equalityConstraintTargetValues_ = Eigen::VectorXd();
    this->equalityConstraintRelativeWeights_ = Eigen::VectorXd();

    for (auto limb : this->limbs_) {
      if (this->getWholeBodyState().getContactFlags()[RD::template mapKeyIdToKeyEnum<LimbEnum>(limb->getLimbUInt())] ==
          ContactStateEnum::ContactOpen) {
        const auto& desiredState = limb->getEndEffector().getStateDesired(loco::TimePoint::Now, this->endEffectorFrame_);
        const auto& measuredState = limb->getEndEffector().getStateMeasured(loco::TimePoint::Now, this->endEffectorFrame_);

        this->setOptimizationTaskPositionTrackingInSourceFrame(
            solutionSpaceDimension, MotionTaskFrame::WORLD,
            desiredState.getPositionWorldToEndEffectorInWorldFrame().toImplementation(),
            desiredState.getLinearVelocityEndEffectorInWorldFrame().toImplementation(),
            desiredState.getLinearAccelerationEndEffectorInWorldFrame().toImplementation(),
            measuredState.getPositionWorldToEndEffectorInWorldFrame().toImplementation(),
            measuredState.getLinearVelocityEndEffectorInWorldFrame().toImplementation(),
            measuredState.getTranslationJacobianWorldToEndEffectorInWorldFrame(),
            measuredState.getTranslationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame(), appendToTask);
        appendToTask = true;
      }
    }

    return true;
  }
};

}  // namespace whole_body_control_romo
