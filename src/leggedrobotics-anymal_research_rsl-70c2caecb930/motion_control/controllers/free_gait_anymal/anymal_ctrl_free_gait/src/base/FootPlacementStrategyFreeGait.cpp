/*
 * FootPlacementStrategyFreeGait.hpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "anymal_ctrl_free_gait/base/FootPlacementStrategyFreeGait.hpp"
#include "message_logger/message_logger.hpp"
#include "tinyxml_tools/tinyxml_tools.hpp"

#include <tuple>

namespace loco {

FootPlacementStrategyFreeGait::FootPlacementStrategyFreeGait(loco_anymal::LegsAnymal& legs, TorsoBase& torso,
                                                             anymal_model::AnymalModel& anymalModel,
                                                             anymal_model::AnymalModel& anymalModelDesired, TerrainModelBase& terrain,
                                                             WholeBody& wholeBody, free_gait::Executor& executor)
    : FootPlacementStrategyBase(),
      legs_(legs),
      torso_(torso),
      anymalModel_(anymalModel),
      anymalModelDesired_(anymalModelDesired),
      terrain_(terrain),
      wholeBody_(wholeBody),
      executor_(executor),
      swingTrajectoryGenerator_(),
      maxRegainContactDistance_(0.0),
      endSpeedRegainContact_(0.0) {
  isLegInRegainMode_.insert({AD::LimbEnum::LF_LEG, false});
  isLegInRegainMode_.insert({AD::LimbEnum::RF_LEG, false});
  isLegInRegainMode_.insert({AD::LimbEnum::LH_LEG, false});
  isLegInRegainMode_.insert({AD::LimbEnum::RH_LEG, false});
}

bool FootPlacementStrategyFreeGait::loadParameters(const TiXmlHandle& handle) {
  const TiXmlHandle fpsHandle = tinyxml_tools::getChildHandle(handle, "FootPlacementStrategy");
  const TiXmlHandle regainContactHandle = tinyxml_tools::getChildHandle(fpsHandle, "RegainContact");
  tinyxml_tools::loadParameter(maxRegainContactDistance_, regainContactHandle, "maxDistance", 0.2);
  tinyxml_tools::loadParameter(endSpeedRegainContact_, regainContactHandle, "endSpeed", 0.4);
  return true;
}

bool FootPlacementStrategyFreeGait::initialize(double dt) {
  for (auto leg : legs_) {
    leg->getLimbStateDesiredPtr()->setJointPositions(leg->getLimbStateMeasured().getJointPositions());
  }

  for (auto& regainMode : isLegInRegainMode_) {
    regainMode.second = false;
  }

  return true;
}

bool FootPlacementStrategyFreeGait::advance(double dt) {
  updateLegConfigurations();

  for (auto leg : legs_) {
    if (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Motion) {
      generateDesiredLegMotion(leg);
    } else if (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactRecovery) {
      regainContact(leg, dt);
    } else {
      bool& legIsInRegainMode = isLegInRegainMode_[AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt())];
      if (legIsInRegainMode) {
        legIsInRegainMode = false;
        if (executor_.getQueue().active()) {
          executor_.getQueue().skipCurrentStep();
        }
      }
    }
  }

  // The other states are handled elsewhere.
  return true;
}

void FootPlacementStrategyFreeGait::updateLegConfigurations() {
  // Check leg configuration based on knee angle.
  bool isLeftForeLegBentNormal = legs_.getLeftForeLeg().getLimbStateMeasured().getJointPositions()(2) <= 0.0;
  bool isRightForeLegBentNormal = legs_.getRightForeLeg().getLimbStateMeasured().getJointPositions()(2) <= 0.0;
  bool isLeftHindLegBentNormal = legs_.getLeftHindLeg().getLimbStateMeasured().getJointPositions()(2) <= 0.0;
  bool isRightHindLegBentNormal = legs_.getRightHindLeg().getLimbStateMeasured().getJointPositions()(2) <= 0.0;
  anymal_model::LegConfigurations legConfigurations(isLeftForeLegBentNormal, isRightForeLegBentNormal, isLeftHindLegBentNormal,
                                                    isRightHindLegBentNormal);
  anymalModel_.setLegConfigurations(legConfigurations);
}

void FootPlacementStrategyFreeGait::generateDesiredLegMotion(LegBase* leg) {
  const auto limbEnum = AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt());

  const auto& controlSetup = executor_.getState().getControlSetup(limbEnum);

  const bool positionSetup = controlSetup.at(free_gait::ControlLevel::Position);
  const bool velocitySetup = controlSetup.at(free_gait::ControlLevel::Velocity);
  const bool accelerationSetup = controlSetup.at(free_gait::ControlLevel::Acceleration);
  const bool effortSetup = controlSetup.at(free_gait::ControlLevel::Effort);

  // Reset.
  loco::JointControlModes jointControlModes = leg->getInitializedJointControlModes();
  jointControlModes.setConstant(ControlMode::MODE_FREEZE);

  loco::JointPositions jointPositions;
  leg->populateJointPositions(jointPositions);

  loco::JointVelocities jointVelocities;
  leg->populateJointVelocities(jointVelocities);

  loco::JointTorques jointTorques;
  leg->populateJointTorques(jointTorques);

  // Control mode.
  if (positionSetup && !velocitySetup && !accelerationSetup && !effortSetup) {
    jointControlModes.setConstant(ControlMode::MODE_JOINT_POSITION);
  } else if (positionSetup && velocitySetup && !accelerationSetup && !effortSetup) {
    jointControlModes.setConstant(ControlMode::MODE_JOINT_POSITION_VELOCITY);
  } else if (effortSetup) {
    jointControlModes.setConstant(ControlMode::MODE_JOINT_TORQUE);
  } else if (positionSetup && velocitySetup && accelerationSetup) {
    jointControlModes.setConstant(ControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS);
  } else if (positionSetup || velocitySetup || accelerationSetup || effortSetup) {
    MELO_WARN("Unknown control setup,")
  }

  // Values.
  if (positionSetup) {
    jointPositions = loco::JointPositions(executor_.getState().getJointPositionsForLimb(limbEnum).vector());
  }

  if (velocitySetup) {
    jointVelocities = loco::JointVelocities(executor_.getState().getJointVelocitiesForLimb(limbEnum).vector());
  }

  if (accelerationSetup) {
    // q_dotdot.
    anymal_model::JointAccelerationsLimb jointAccelerations = executor_.getState().getJointAccelerationsForLimb(limbEnum);
    Force endEffectorForceInWorldFrame = Force::Zero();

    // Compute joint torques based on feedforward acceleration.
    const auto startIndex = AD::getBranchStartIndexInU(AD::mapKeyIdToKeyEnum<AD::BranchEnum>(leg->getBranchUInt()));
    constexpr auto numDofLimb = AD::getNumDofLimb();

    // Mass inertial matrix (M).
    const Eigen::MatrixXd massInertiaMatrix(wholeBody_.getWholeBodyMassMatrix().block<numDofLimb, numDofLimb>(startIndex, startIndex));

    // Inverse dynamics (tau = M * q_dotdot + h).
    jointTorques += loco::JointTorques(massInertiaMatrix * jointAccelerations.vector());
  }

  if (effortSetup) {
    jointTorques = loco::JointTorques(executor_.getState().getJointEffortsForLimb(limbEnum).vector());
    auto endEffectorForceInWorldFrame = Force::Zero();

    // Friction compensation.
    if (velocitySetup) {
      const auto desiredLinearVelocityDirection = executor_.getState().getEndEffectorVelocityInWorldFrame(limbEnum).vector().normalized();
      endEffectorForceInWorldFrame += Force(executor_.getState().getFeedForwardFrictionNorm() * desiredLinearVelocityDirection);
    }

    // Impedance Controller Feedback (follow end-effector trajectory).
    if (positionSetup) {
      Position positionErrorInWorldFrame = leg->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() -
                                           executor_.getState().getEndEffectorPositionInWorldFrame(limbEnum);
      const auto& gain = executor_.getState().getImpedanceGainInWorldFrame(free_gait::ImpedanceControl::Position);
      endEffectorForceInWorldFrame -= Force(gain.vector().cwiseProduct(positionErrorInWorldFrame.vector()));
    }

    if (velocitySetup) {
      LinearVelocity velocityErrorInWorldFrame = leg->getEndEffector().getStateMeasured().getLinearVelocityEndEffectorInWorldFrame() -
                                                 executor_.getState().getEndEffectorVelocityInWorldFrame(limbEnum);
      const auto& gain = executor_.getState().getImpedanceGainInWorldFrame(free_gait::ImpedanceControl::Velocity);
      endEffectorForceInWorldFrame -= Force(gain.vector().cwiseProduct(velocityErrorInWorldFrame.vector()));
    } else {
      // Damp in case that there is no contact (safety).
      const auto& gain = executor_.getState().getImpedanceGainInWorldFrame(free_gait::ImpedanceControl::Velocity);
      const auto& linearVelocity = leg->getEndEffector().getStateMeasured().getLinearVelocityEndEffectorInWorldFrame();
      endEffectorForceInWorldFrame -= Force(gain.vector().cwiseProduct(linearVelocity.vector()));
    }

    // Impedance Controller Feedforward (end-effector force).
    if (positionSetup || velocitySetup) {
      const auto& gain = executor_.getState().getImpedanceGainInWorldFrame(free_gait::ImpedanceControl::Force);
      endEffectorForceInWorldFrame +=
          Force(gain.vector().cwiseProduct(executor_.getState().getEndEffectorForceInWorldFrame(limbEnum).vector()));
    } else {
      endEffectorForceInWorldFrame += executor_.getState().getEndEffectorForceInWorldFrame(limbEnum);
    }

    // Add to joint torques.
    const auto& orientationWorldToBase = wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase();
    const Force endEffectorForceInBaseFrame = orientationWorldToBase.rotate(endEffectorForceInWorldFrame);
    jointTorques += loco::JointTorques(leg->getFoot().getStateMeasured().getTranslationJacobianBaseToEndEffectorInBaseFrame().transpose() *
                                       endEffectorForceInBaseFrame.vector());
  }
  // Safety.
  //  if (!positionSetup)
  //    jointPositions = LegBase::JointPositions(executor_.getState().getJointPositions(leg->getLimbEnum()).vector());

  if (accelerationSetup || effortSetup) {
    const auto startIndex = AD::getBranchStartIndexInU(AD::mapKeyIdToKeyEnum<AD::BranchEnum>(leg->getBranchUInt()));
    constexpr auto numDofLimb = AD::getNumDofLimb();
    // Non-linear terms from EOM (h).
    const Eigen::VectorXd nonlinearTerms(wholeBody_.getWholeBodyNonlinearEffects().segment<numDofLimb>(startIndex));
    jointTorques += loco::JointTorques(nonlinearTerms);
  }

  // Writing control mode and values.
  leg->getLimbStateDesiredPtr()->setJointControlModes(jointControlModes);
  leg->getLimbStateDesiredPtr()->setJointPositions(jointPositions);
  leg->getLimbStateDesiredPtr()->setJointVelocities(jointVelocities);
  leg->getLimbStateDesiredPtr()->setJointTorques(jointTorques);
}

void FootPlacementStrategyFreeGait::regainContact(LegBase* leg, const double dt) {
  const auto limbEnum = AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt());

  if (!isLegInRegainMode_[limbEnum]) {
    executor_.addToFeedback("No contact detected, adding second regain contact step.");
    isLegInRegainMode_[limbEnum] = true;

    const Position& startPosition = leg->getPositionWorldToLostContactPositionInWorldFrame();
    free_gait::EndEffectorTarget legMotion(limbEnum);
    Position targetPosition(startPosition);
    targetPosition.z() -= maxRegainContactDistance_;
    legMotion.setTargetPosition(executor_.getAdapter().getWorldFrameId(), targetPosition);
    LinearVelocity targetSpeed;
    targetSpeed.z() = -endSpeedRegainContact_;
    legMotion.setTargetVelocity(executor_.getAdapter().getWorldFrameId(), targetSpeed);
    legMotion.setAverageVelocity(targetSpeed.norm());
    if (executor_.getState().hasSurfaceNormal(limbEnum)) {
      legMotion.setSurfaceNormal(executor_.getState().getSurfaceNormal(limbEnum));
    }
    free_gait::Step regainStep;
    regainStep.addLegMotion(legMotion);
    free_gait::BaseAuto baseAuto;
    baseAuto.setTolerateFailingOptimization(true);
    regainStep.addBaseMotion(baseAuto);

    if (executor_.getQueue().active()) {
      executor_.getQueue().getCurrentStep().reset();
    }
    executor_.getQueue().addInFront(regainStep);
  }

  if (executor_.advance(dt, true)) {
    generateDesiredLegMotion(leg);
  }

  // TODO(pfankhauser) Add target velocity vector based on fitted terrain for footstep.
  //  // Get normal to move foot along.
  //  loco::Vector normal = loco::Vector::UnitZ();
  //
  //  if (executor_.getState().hasSurfaceNormal(limbEnum)) {
  //    normal = executor_.getState().getSurfaceNormal(limbEnum);
  //  } else {
  //    if (!terrain_.getNormal(startPosition, normal)) {
  //      throw std::runtime_error("FootPlacementStrategyFreeGait::regainContact cannot get terrain normal.");
  //    }
  //  }
  //
  //  // Check if within joint limits.
  //  loco::Position positionHipToFoot = leg->getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame() -
  //  leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame(); if (positionHipToFoot.norm() >=
  //  maxLegLengthForRegainingContact_) return;
}

bool FootPlacementStrategyFreeGait::setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1,
                                                      const FootPlacementStrategyBase& footPlacementStrategy2, double t) {
  throw std::runtime_error("FootPlacementStrategyFreeGait::setToInterpolated not implemented yet!");
  return false;
}

const loco_anymal::LegsAnymal& FootPlacementStrategyFreeGait::getLegs() const {
  return legs_;
}

const SwingTrajectoryGeneratorBase& FootPlacementStrategyFreeGait::getSwingTrajectoryGenerator() const {
  return swingTrajectoryGenerator_;
}

SwingTrajectoryGeneratorBase* FootPlacementStrategyFreeGait::getSwingTrajectoryGeneratorPtr() {
  return &swingTrajectoryGenerator_;
}

}  // namespace loco
