/*!
 * @file     VirtualModelController.hpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso
 * @date     Aug 6, 2013
 * @brief
 */

// loco
#include "loco/motion_control/VirtualModelControllerContactInvariantDamper.hpp"

namespace loco {

VirtualModelControllerContactInvariantDamper::VirtualModelControllerContactInvariantDamper(
    WholeBody& wholeBody, ContactForceDistributionInterface& contactForceDistribution)
    : VirtualModelController(wholeBody, contactForceDistribution), contactInvariantDamper_(wholeBody) {}

bool VirtualModelControllerContactInvariantDamper::addVariablesToLog(const std::string& ns) const {
  contactInvariantDamper_.addVariablesToLog(ns);
  return VirtualModelController::addVariablesToLog(ns);
}

bool VirtualModelControllerContactInvariantDamper::addParametersToHandler(const std::string& ns) {
  contactInvariantDamper_.addParametersToHandler(ns);
  return VirtualModelController::addParametersToHandler(ns);
}

bool VirtualModelControllerContactInvariantDamper::initialize(double dt) {
  if (!contactInvariantDamper_.initialize(dt)) {
    return false;
  }
  return VirtualModelController::initialize(dt);
}

bool VirtualModelControllerContactInvariantDamper::advance(double dt) {
  bool success = true;

  success &= setControlModeForLimbs();
  success &= computeError();
  success &= computeGravityCompensation();
  success &= computeVirtualForce(dt);
  success &= computeVirtualTorque(dt);

  if (!contactForceDistribution_.computeForceDistribution(virtualForceInBaseFrame_, virtualTorqueInBaseFrame_)) {
    MELO_WARN("[VirtualModelController::advance] Computation of contact force distribution returned false!");
    return false;
  }

  // Set torques from desired contact forces
  if (!jacobianTranspose_.advance(dt)) {
    MELO_WARN("Loco: jacobianTranspose_->advance() returned false!");
    return false;
  }

  if (!contactInvariantDamper_.advance(dt)) {
    MELO_WARN("Loco: contactInvariantDamper_->advance() returned false!");
    return false;
  }

  //  if (supportLegControlMode_ == loco::ControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE) {
  jacobianTranspose_.setJointPositionsFromDesiredBase();
  //  }

  return success;
}

bool VirtualModelControllerContactInvariantDamper::loadParameters(const TiXmlHandle& handle) {
  if (!contactInvariantDamper_.loadParameters(handle)) {
    return false;
  }

  return VirtualModelController::loadParameters(handle);
}

} /* namespace loco */
