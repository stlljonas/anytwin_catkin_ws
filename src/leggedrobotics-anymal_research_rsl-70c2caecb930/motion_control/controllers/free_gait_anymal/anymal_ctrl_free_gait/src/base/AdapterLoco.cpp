/*
 * AdapterLoco.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "anymal_ctrl_free_gait/base/AdapterLoco.hpp"
#include "free_gait_core/leg_motion/EndEffectorMotionBase.hpp"

#include <robot_utils/math/LinearAlgebra.hpp>
#include <robot_utils/math/math.hpp>

namespace free_gait {

AdapterLoco::AdapterLoco(anymal_model::AnymalModel& anymalModel, anymal_model::AnymalModel& anymalModelDesired, loco::WholeBody& wholeBody,
                         const anymal_motion_control::State& robotState)
    : AdapterAnymal(anymalModel),
      anymalModelDesired_(anymalModelDesired),
      wholeBody_(wholeBody),
      robotState_(robotState),
      legs_(*wholeBody.getLegsPtr()),
      startUnloadingLegAtPhase_(0.0),
      loadFactorLowerBound_(0.0) {}

bool AdapterLoco::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle handleToTerrainParameters(handle.FirstChild("TorsoControl").FirstChild("LoadFactor"));

  TiXmlElement* pElem = handleToTerrainParameters.Element();
  if (pElem == nullptr) {
    printf("[AdapterLoco::loadParameters] Could not find TorsoControl::TorsoControl section in parameter file.\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("startUnloadingAtPhase", &startUnloadingLegAtPhase_) != TIXML_SUCCESS) {
    printf("[AdapterLoco::loadParameters] Could not find TorsoControl::startUnloadingAtPhase in parameter file.\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("lowerBound", &loadFactorLowerBound_) != TIXML_SUCCESS) {
    printf("[AdapterLoco::loadParameters] Could not find TorsoControl::lowerBound in parameter file.\n");
    return false;
  }

  return true;
}

bool AdapterLoco::resetExtrasWithRobot(const StepQueue& stepQueue, State& state) {
  AdapterAnymal::resetExtrasWithRobot(stepQueue, state);
  try {
    auto& stateLoco = dynamic_cast<StateLoco&>(state);
    for (const auto limb : limbs_) {
      const bool isSupportLeg = stateLoco.isSupportLeg(limb);
      // Resetting gait pattern.
      const double max = std::numeric_limits<double>::max();
      stateLoco.setSwingPhase(limb, isSupportLeg ? -1.0 : 0.0);
      stateLoco.setSwingDuration(limb, isSupportLeg ? 0.0 : max);

      // Resetting load factor.
      const double loadFactor = isSupportLeg ? 1.0 : 0.0;
      stateLoco.setLegLoadFactor(limb, loadFactor);
    }
  } catch (std::bad_cast& bc) {
    MELO_ERROR_STREAM("[AdapterLoco::resetExtrasWithRobot] Could not cast state!");
    return false;
  }

  anymalModelDesired_.setState(state, true, true, false);
  return true;
}

bool AdapterLoco::updateExtrasBefore(const StepQueue& stepQueue, State& state) {
  // We do this, because some other modules might mess with the desired state.
  anymalModelDesired_.setState(state, true, true, false);
  AdapterAnymal::updateExtrasBefore(stepQueue, state);
  return true;
}

bool AdapterLoco::updateExtrasAfter(const StepQueue& stepQueue, State& state) {
  anymalModelDesired_.setState(state, true, true, false);

  // Update torso desired state.
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setPositionWorldToBaseInWorldFrame(state.getPositionWorldToBaseInWorldFrame());
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setOrientationEulerAnglesZyxBaseToWorld(
      loco::EulerAnglesZyx(state.getOrientationBaseToWorld()).getUnique());

  try {
    auto& stateLoco = dynamic_cast<StateLoco&>(state);
    updateGaitPattern(stepQueue, stateLoco);
    updateLegLoadFactor(stepQueue, stateLoco);
    updateLog(stepQueue, stateLoco);
  } catch (const std::bad_cast& bc) {
    MELO_ERROR_STREAM("[AdapterLoco::updateExtrasAfter] Could not cast state!");
    return false;
  }
  return true;
}

bool AdapterLoco::updateGaitPattern(const StepQueue& stepQueue, StateLoco& state) const {
  if (!stepQueue.active()) {
    return true;
  }

  const Step& step = stepQueue.getCurrentStep();
  for (const auto limb : limbs_) {
    if (step.hasLegMotion(limb)) {
      state.setSwingPhase(limb, step.getLegMotionPhase(limb));
      state.setSwingDuration(limb, step.getLegMotionDuration(limb));
    } else {
      state.setSwingPhase(limb, -1.0);
      state.setSwingDuration(limb, 0.0);
    }
  }
  return true;
}

bool AdapterLoco::updateLegLoadFactor(const StepQueue& stepQueue, StateLoco& state) const {
  for (const auto limb : limbs_) {
    if (!state.isSupportLeg(limb)) {
      // Swing leg, nothing to do.
      state.setLegLoadFactor(limb, 0.0);
      continue;
    }

    // Stance leg.
    double loadFactor = 1.0;
    if (stepQueue.size() > 1 && stepQueue.active()) {
      if (stepQueue.getNextStep().hasLegMotion(limb)) {
        loadFactor = 1.0 - (1.0 - loadFactorLowerBound_) *
                               robot_utils::mapTo01Range(stepQueue.getCurrentStep().getTotalPhase(), startUnloadingLegAtPhase_, 1.0);
      }
    }

    state.setLegLoadFactor(limb, loadFactor);
  }

  return true;
}

bool AdapterLoco::updateLog(const StepQueue& stepQueue, StateLoco& state) const {
  if (!stepQueue.active()) {
    return true;
  }

  const auto& step = stepQueue.getCurrentStep();
  for (const auto limb : limbs_) {
    if (step.hasLegMotion(limb)) {
      const auto& legMotion = step.getLegMotion(limb);
      if (legMotion.getTrajectoryType() == LegMotionBase::TrajectoryType::EndEffector) {
        try {
          const auto& endEffectorMotion = dynamic_cast<const EndEffectorMotionBase&>(legMotion);
          if (endEffectorMotion.getControlSetup().at(free_gait::ControlLevel::Position)) {
            const Position position = transformPosition(endEffectorMotion.getFrameId(free_gait::ControlLevel::Position), getWorldFrameId(),
                                                        endEffectorMotion.evaluatePosition(step.getTime()));
            legs_.getPtr(AD::mapKeyEnumToKeyId(limb))
                ->getFootPtr()
                ->getStateDesiredPtr()
                ->setPositionWorldToEndEffectorInWorldFrame(position);
          }
          if (endEffectorMotion.getControlSetup().at(free_gait::ControlLevel::Velocity)) {
            const LinearVelocity velocity = transformLinearVelocity(endEffectorMotion.getFrameId(free_gait::ControlLevel::Velocity),
                                                                    getWorldFrameId(), endEffectorMotion.evaluateVelocity(step.getTime()));
            legs_.getPtr(AD::mapKeyEnumToKeyId(limb))
                ->getFootPtr()
                ->getStateDesiredPtr()
                ->setLinearVelocityEndEffectorInWorldFrame(velocity);
          }
        } catch (...) {
          MELO_ERROR_STREAM("[AdapterLoco::updateLog] Could not cast leg motion!");
          return false;
        }
        continue;
      }
    }
  }

  return true;
}

bool AdapterLoco::isExecutionOk() const {
  for (const auto& leg : legs_) {
    switch (leg->getStateSwitcher().getState()) {
      case (loco::StateSwitcher::States::StanceNormal):
      case (loco::StateSwitcher::States::SwingNormal):
      case (loco::StateSwitcher::States::SwingLateLiftOff):
      case (loco::StateSwitcher::States::SwingEarlyTouchDown):
      case (loco::StateSwitcher::States::SwingBumpedIntoObstacle):
        continue;
        break;

      case (loco::StateSwitcher::States::StanceSlipping):
      case (loco::StateSwitcher::States::StanceLostContact):
      case (loco::StateSwitcher::States::SwingExpectingContact):
        if (leg->getLimbStrategyPtr()->getLimbStrategyEnum() == loco::LimbStrategyEnum::ContactRecovery) {
          return true;
        }
        return false;
        break;

      default:
        std::cerr << "AdapterLoco::isExecutionOk(): Unhandled state switcher mode!" << std::endl;
        return false;
        break;
    }
  }
  return true;
}

bool AdapterLoco::isLegGrounded(const LimbEnum& limb) const {
  return legs_.get(AD::mapKeyEnumToKeyId(limb)).getContactSchedule().isGrounded();
}

ControlSetup AdapterLoco::getControlSetup(const BranchEnum& branch) const {
  ControlSetup controlSetup;
  controlSetup[ControlLevel::Position] = false;
  controlSetup[ControlLevel::Velocity] = false;
  controlSetup[ControlLevel::Acceleration] = false;
  controlSetup[ControlLevel::Effort] = false;

  if (branch == BranchEnum::BASE) {
    // TODO(pfankhauser): Extend with velocity and force control options.
    controlSetup[ControlLevel::Position] = true;
  } else {
    unsigned int limbUInt = AD::mapKeyEnumToKeyId<AD::BranchEnum, AD::LimbEnum>(branch);
    const loco::JointControlModes jointControlModes = legs_.getPtr(limbUInt)->getLimbStateDesired().getJointControlModes();
    switch (jointControlModes(0)) {
      case loco::ControlMode::MODE_JOINT_POSITION:
        controlSetup[ControlLevel::Position] = true;
        break;
      case loco::ControlMode::MODE_JOINT_VELOCITY:
        controlSetup[ControlLevel::Velocity] = true;
        break;
      case loco::ControlMode::MODE_JOINT_TORQUE:
        controlSetup[ControlLevel::Effort] = true;
        break;
      case loco::ControlMode::MODE_JOINT_POSITION_VELOCITY:
        controlSetup[ControlLevel::Position] = true;
        controlSetup[ControlLevel::Velocity] = true;
        break;
      case loco::ControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE:
        controlSetup[ControlLevel::Position] = true;
        controlSetup[ControlLevel::Effort] = true;
        break;
      case loco::ControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS:
        controlSetup[ControlLevel::Position] = true;
        controlSetup[ControlLevel::Effort] = true;
        break;
      case loco::ControlMode::MODE_FREEZE:
        controlSetup[ControlLevel::Position] = true;  // Hack.
        break;
      default:
        MELO_WARN("Unhandled joint control mode [%s].", loco::getControlModeKeys()[jointControlModes(0)].getName())
        break;
    }
  }

  return controlSetup;
}

JointVelocitiesLeg AdapterLoco::getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame(
    const LimbEnum& limb, const LinearVelocity& endEffectorLinearVelocityInWorldFrame) const {
  const LinearVelocity linearVelocityBaseInBaseFrame = getOrientationBaseToWorld().inverseRotate(getLinearVelocityBaseInWorldFrame());

  const LinearVelocity linearVelocityBaseToFootInBaseFrame =
      getOrientationBaseToWorld().inverseRotate(endEffectorLinearVelocityInWorldFrame) - linearVelocityBaseInBaseFrame -
      LinearVelocity(
          getAngularVelocityBaseInBaseFrame().toImplementation().cross(getPositionBaseToFootInBaseFrame(limb).toImplementation()));

  const JointVelocitiesLeg jointVelocitiesLeg(
      legs_.getPtr(AD::mapKeyEnumToKeyId(limb))
          ->getEndEffectorPtr()
          ->getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(linearVelocityBaseToFootInBaseFrame));

  return jointVelocitiesLeg;
}

JointAccelerationsLeg AdapterLoco::getJointAccelerationsFromEndEffectorLinearAccelerationInWorldFrame(
    const LimbEnum& limb, const LinearAcceleration& endEffectorLinearAccelerationInWorldFrame) const {
  // TODO(pfankhauser): Add proper inclusions of base linear and rotational acceleration.

  const LinearAcceleration linearAccelerationBaseInBaseFrame =
      getOrientationBaseToWorld().inverseRotate(getLinearAccelerationBaseInWorldFrame());

  //
  //  const LinearVelocity linearVelocityBaseToFootInBaseFrame = getOrientationBaseToWorld()
  //      .inverseRotate(endEffectorLinearVelocityInWorldFrame) - linearVelocityBaseInBaseFrame
  //      - LinearVelocity(getAngularVelocityBaseInBaseFrame().toImplementation().cross(
  //              getPositionBaseToFootInBaseFrame(limb).toImplementation()));
  //

  // Desired linear acceleration of the foot in base frame (x_dotdot_desired).
  LinearAcceleration linearAccelerationBaseToFootInBaseFrame =
      getOrientationBaseToWorld().inverseRotate(endEffectorLinearAccelerationInWorldFrame) -
      linearAccelerationBaseInBaseFrame;  // TODO(pfankhauser): Add rotation effect.

  // Actual joint velocities (q_dot).
  const auto jointVelocities = getJointVelocitiesForLimb(limb);

  const auto endEffector = legs_.getPtr(AD::mapKeyEnumToKeyId(limb))->getEndEffectorPtr();
  // Jacobian base to end effector (J).
  const auto jacobian = endEffector->getStateMeasured().getTranslationJacobianBaseToEndEffectorInBaseFrame();
  const auto jacobianInverse = robot_utils::pseudoInverseAdaptiveDls(jacobian);
  // Time derivative of Jacobian base to end effector (J_dot).
  const auto jacobianTimeDerivative = endEffector->getStateMeasured().getTranslationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame();
  // Desired joint accelerations (q_dotdot_desired = J_inverse * (x_dotdot - J_dot * q_dot).
  const JointAccelerationsLeg jointAccelerations = JointAccelerationsLeg(
      jacobianInverse * (linearAccelerationBaseToFootInBaseFrame.vector() - jacobianTimeDerivative * jointVelocities.vector()));

  return jointAccelerations;
}

LinearVelocity AdapterLoco::getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb, const JointVelocitiesLeg& jointVelocities,
                                                                            const std::string& frameId) const {
  const auto endEffector = legs_.getPtr(AD::mapKeyEnumToKeyId(limb))->getEndEffectorPtr();
  // Jacobian base to end effector (J).
  const auto jacobian = endEffector->getStateMeasured().getTranslationJacobianBaseToEndEffectorInBaseFrame();
  // End effector velocity relative to base expressed in base frame.
  const LinearVelocity endEffectorLinearVelocityInBaseFrame = LinearVelocity(jacobian * jointVelocities.vector());

  if (frameId == getBaseFrameId()) {
    // Velocity relative to base.
    return endEffectorLinearVelocityInBaseFrame;
  } else {
    // Velocity relative to world.
    const LinearVelocity inducedLinearVelocityInBaseFrame(
        getAngularVelocityBaseInBaseFrame().vector().cross(getPositionBaseToFootInBaseFrame(limb).vector()));
    const LinearVelocity linearVelocityInWorldFrame =
        getLinearVelocityBaseInWorldFrame() +
        transformLinearVelocity(getBaseFrameId(), getWorldFrameId(), inducedLinearVelocityInBaseFrame) +
        transformLinearVelocity(getBaseFrameId(), getWorldFrameId(), endEffectorLinearVelocityInBaseFrame);
    return transformLinearVelocity(getWorldFrameId(), frameId, linearVelocityInWorldFrame);
  }
}

loco::WholeBody* AdapterLoco::getWholeBodyPtr() const {
  return &wholeBody_;
}

} /* namespace free_gait */
