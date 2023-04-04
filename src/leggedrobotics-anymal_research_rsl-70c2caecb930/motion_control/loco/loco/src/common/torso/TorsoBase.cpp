/*
 * StateBase.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring
 */

#include "loco/common/torso/TorsoBase.hpp"
#include "signal_logger/signal_logger.hpp"

namespace loco {

TorsoBase::TorsoBase(const std::string& name, TorsoPropertiesPtr&& properties)
    : ModuleBase(name), torsoProperties_(std::move(properties)), stridePhase_(0.0), strideDuration_(0.0) {
  torsoStateDesired_.reset(new TorsoStateDesired());
  torsoStateMeasured_.reset(new TorsoStateMeasured());
}

void TorsoBase::print(std::ostream& out) const {
  out << "Desired state:" << std::endl << getDesiredState() << std::endl;
  out << "Measured state:" << std::endl << getMeasuredState();
}

bool TorsoBase::addVariablesToLog(const std::string& /* ns */) const {
  //-- Errors
  signal_logger::add(getDesiredState().getPositionErrorInControlFrame(), "/torso/positionErrorInControlFrame", "/loco", "m");

  signal_logger::add(getDesiredState().getLinearVelocityErrorInControlFrame(), "/torso/linearVelocityErrorInControlFrame", "/loco", "m/s");
  //--

  //-- Desired velocities
  signal_logger::add(getDesiredState().getLinearVelocityTargetInControlFrame(), "/torso/desiredLinearVelocityBaseInControlFrame", "/loco",
                     "m/s");
  signal_logger::add(getDesiredState().getAngularVelocityBaseInControlFrame(), "/torso/desiredAngularVelocityBaseInControlFrame", "/loco",
                     "rad/s");

  const std::string logNameSpace = "/loco/";

  /* FIXME signal_logger has a KindrVectorAtPositionType
    signal_logger::add(getDesiredState().getLinearVelocityTargetInControlFrame(),
                       getDesiredState().getPositionWorldToBaseInWorldFrame(),
                       "/torso/desiredLinearVelocityBaseInControlFrame",
                       "control",
                       "odom",
                       logNameSpace,
                       "m/s");
  */
  //--

  //-- Measured pose
  signal_logger::add(getMeasuredState().inControlFrame().getPositionControlToBaseInControlFrame(),
                     "/torso/measPositionControlToBaseInControlFrame", "/loco", "m");

  signal_logger::add(getMeasuredState().getPositionWorldToBaseInWorldFrame(), "/torso/measPositionWorldToBaseInWorldFrame", "/loco", "m");
  signal_logger::add(getMeasuredState().inControlFrame().getOrientationWorldToControl(), "/torso/measOrientationWorldToControl", "/loco");
  signal_logger::add(getMeasuredState().getOrientationEulerAnglesZyxBaseToWorld(), "/torso/measEulerAnglesZyxBaseToWorld", "/loco", "rad");

  signal_logger::add(getMeasuredState().inControlFrame().getOrientationEulerAnglesZyxControlToBase(),
                     "/torso/measEulerAnglesZyxControlToBase", "/loco", "rad");

  //-- Measured velocities
  signal_logger::add(getMeasuredState().inControlFrame().getLinearVelocityBaseInControlFrame(),
                     "/torso/measLinearVelocityBaseInControlFrame", "/loco", "m/s");
  signal_logger::add(getMeasuredState().inControlFrame().getAngularVelocityBaseInControlFrame(),
                     "/torso/measAngularVelocityBaseInControlFrame", "/loco", "rad/s");
  //--

  //-- Desired pose
  signal_logger::add(getDesiredState().getPositionControlToTargetInControlFrame(), "/torso/desPositionControlToTargetInControlFrame",
                     "/loco", "m");

  signal_logger::add(getDesiredState().getPositionWorldToBaseInWorldFrame(), "/torso/desPositionWorldToBaseInWorldFrame", "/loco", "m");

  signal_logger::add(getDesiredState().getOrientationEulerAnglesZyxControlToBase(), "/torso/desEulerAnglesZyxControlToBase", "/loco",
                     "rad");
  signal_logger::add(getDesiredState().getOrientationEulerAnglesZyxBaseToWorld(), "/torso/desEulerAnglesZyxBaseToWorld", "/loco", "rad");

  signal_logger::add(getDesiredState().getLinearAccelerationTargetInControlFrame(), "/torso/desLinearAccelerationTargetInControlFrame",
                     "/loco", "m");

  signal_logger::add(stridePhase_, "/torso/stridePhase", "/loco", "percentage");

  signal_logger::add(strideDuration_, "/torso/strideDuration", "/loco", "s");

  return true;
}

} /* namespace loco */
