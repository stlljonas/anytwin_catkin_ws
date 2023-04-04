/*!
 * @file     VirtualModelController.hpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso
 * @date     Aug 6, 2013
 * @brief
 */

// loco
#include "loco/motion_control/VirtualModelController.hpp"
#include "loco/common/loco_common.hpp"
#include "loco/common/topology_conversions.hpp"
#include "loco/common/typedefs.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

// kindr
#include <kindr/Core>

// message logger
#include <message_logger/message_logger.hpp>

namespace loco {

VirtualModelController::VirtualModelController(WholeBody& wholeBody, ContactForceDistributionInterface& contactForceDistribution)
    : MotionControllerBase(wholeBody),
      contactForceDistribution_(contactForceDistribution),
      jacobianTranspose_(wholeBody),
      positionErrorInControlFrame_(),
      orientationError_(),
      linearVelocityErrorInControlFrame_(),
      angularVelocityErrorInControlFrame_(),
      virtualIntegralForceInControlFrame_(),
      virtualIntegralForceLimit_(),
      virtualIntegralTorqueInBaseFrame_(),
      virtualIntegralTorqueLimit_(),
      gravityCompensationForceInBaseFrame_(),
      gravityCompensationTorqueInBaseFrame_(),
      gravityCompensationForcePercentage_(1.0),
      virtualForceInBaseFrame_(),
      virtualTorqueInBaseFrame_(),
      isIntegratingTheErrors_(true),
      offsetForceInBaseFrame_(),
      offsetTorqueInBaseFrame_(),
      proportionalGainTranslation_(500.0 * Eigen::Vector3d::Ones(), 0.0 * Eigen::Vector3d::Ones(), 2000.0 * Eigen::Vector3d::Ones()),
      derivativeGainTranslation_(100.0 * Eigen::Vector3d::Ones(), 0.0 * Eigen::Vector3d::Ones(), 400.0 * Eigen::Vector3d::Ones()),
      proportionalGainRotation_(20.0 * Eigen::Vector3d::Ones(), 0.0 * Eigen::Vector3d::Ones(), 500.0 * Eigen::Vector3d::Ones()),
      derivativeGainRotation_(10.0 * Eigen::Vector3d::Ones(), 0.0 * Eigen::Vector3d::Ones(), 200.0 * Eigen::Vector3d::Ones()),
      integralGainTranslation_(0.0 * Eigen::Vector3d::Ones(), 0.0 * Eigen::Vector3d::Ones(), 1000.0 * Eigen::Vector3d::Ones()),
      feedforwardGainTranslation_(0.0 * Eigen::Vector3d::Ones(), 0.0 * Eigen::Vector3d::Ones(), 1000.0 * Eigen::Vector3d::Ones()),
      integralGainRotation_(0.0 * Eigen::Vector3d::Ones(), 0.0 * Eigen::Vector3d::Ones(), 1000.0 * Eigen::Vector3d::Ones()),
      feedforwardGainRotation_(0.0 * Eigen::Vector3d::Ones(), 0.0 * Eigen::Vector3d::Ones(), 1000.0 * Eigen::Vector3d::Ones()),
      supportLegControlMode_(ControlMode::MODE_FREEZE),
      nonSupportLegControlMode_(ControlMode::MODE_FREEZE) {}

bool VirtualModelController::addVariablesToLog(const std::string& ns) const {
  contactForceDistribution_.addVariablesToLog(ns);

  signal_logger::add(virtualForceInBaseFrame_, "desVirtualForceInBaseFrame", "/loco/vmc/", "N");
  signal_logger::add(virtualTorqueInBaseFrame_, "desVirtualTorqueInBaseFrame", "/loco/vmc/", "Nm");

  signal_logger::add(virtualIntegralForceInControlFrame_, "integratorVirtualForceInBaseFrame", "/loco/vmc/", "N");
  signal_logger::add(virtualIntegralTorqueInBaseFrame_, "integratorVirtualTorqueInBaseFrame", "/loco/vmc/", "Nm");

  return true;
}

bool VirtualModelController::addParametersToHandler(const std::string& ns) {
  parameter_handler::handler->addParam(ns + "vmcKpLin", proportionalGainTranslation_);
  parameter_handler::handler->addParam(ns + "vmcKdLin", derivativeGainTranslation_);
  parameter_handler::handler->addParam(ns + "vmcKiLin", integralGainTranslation_);
  parameter_handler::handler->addParam(ns + "vmcKffLin", feedforwardGainTranslation_);
  parameter_handler::handler->addParam(ns + "vmcKpAng", proportionalGainRotation_);
  parameter_handler::handler->addParam(ns + "vmcKdAng", derivativeGainRotation_);
  parameter_handler::handler->addParam(ns + "vmcKiAng", integralGainRotation_);
  parameter_handler::handler->addParam(ns + "vmcKffAng", feedforwardGainRotation_);
  return true;
}

bool VirtualModelController::initialize(double dt) {
  return jacobianTranspose_.initialize(dt);
}

bool VirtualModelController::setControlModeForLimbs() {
  for (auto limb : limbs_) {
    setControlModeForLimb(limb, supportLegControlMode_, nonSupportLegControlMode_);
  }
  return true;
}

bool VirtualModelController::advance(double dt) {
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

  //  if (supportLegControlMode_ == loco::ControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE) {
  jacobianTranspose_.setJointPositionsFromDesiredBase();
  //  }

  return success;
}

bool VirtualModelController::computeError() {
  const RotationQuaternion& orientationControlToBase = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase();

  positionErrorInControlFrame_ = torso_.getDesiredState().getPositionErrorInControlFrame();
  orientationError_ = -torso_.getDesiredState().getOrientationControlToBase().boxMinus(orientationControlToBase);

  linearVelocityErrorInControlFrame_ = torso_.getDesiredState().getLinearVelocityErrorInControlFrame();
  angularVelocityErrorInControlFrame_ = torso_.getDesiredState().getAngularVelocityBaseInControlFrame() -
                                        torso_.getMeasuredState().inControlFrame().getAngularVelocityBaseInControlFrame();

  return true;
}

bool VirtualModelController::computeGravityCompensation() {
  const LinearAcceleration gravitationalAccelerationInWorldFrame = torso_.getProperties().getGravity();
  LinearAcceleration gravitationalAccelerationInBaseFrame =
      torso_.getMeasuredState().getOrientationWorldToBase().rotate(gravitationalAccelerationInWorldFrame);

  //  gravitationalAccelerationInBaseFrame /= 1.1;

  const Force forceTorso =
      Force(-gravityCompensationForcePercentage_ * torso_.getProperties().getMass() * gravitationalAccelerationInBaseFrame);
  gravityCompensationForceInBaseFrame_ = forceTorso;
  gravityCompensationTorqueInBaseFrame_ = Torque(torso_.getProperties().getBaseToCenterOfMassPositionInBaseFrame().cross(forceTorso));

  for (auto limb : limbs_) {
    const Force forceLimb =
        Force(-gravityCompensationForcePercentage_ * limb->getLimbProperties().getLimbMass() * gravitationalAccelerationInBaseFrame);
    gravityCompensationForceInBaseFrame_ += forceLimb;
    gravityCompensationTorqueInBaseFrame_ += Torque(limb->getLimbProperties().getPositionBaseToLimbComInBaseFrame().cross(forceLimb));
  }
  //  Force gravityCompensationForceWorldFrame_ =
  //  torso_.getMeasuredState().getWorldToBaseOrientationInWorldFrame().inverseRotate(gravityCompensationForce_);
  //  gravityCompensationForceWorldFrame_.x() = 0.0;
  //  gravityCompensationForceWorldFrame_.y() = 0.0;
  //  gravityCompensationForce_ =
  //  torso_.getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(gravityCompensationForceWorldFrame_);

  return true;
}

bool VirtualModelController::computeVirtualForce(double dt) {
  const RotationQuaternion& orientationControlToBase = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase();
  const RotationQuaternion& orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();

  const double totalMass = wholeBody_.getWholeBodyProperties().getTotalMass();

  Eigen::Vector3d feedforwardTermInControlFrame = Eigen::Vector3d::Zero();
  switch (torso_.getDesiredState().getTargetPoint()) {
    case (TorsoStateDesired::TargetPoint::BASE): {
      feedforwardTermInControlFrame =
          torso_.getProperties().getMass() * torso_.getDesiredState().getLinearAccelerationTargetInControlFrame().vector();
    } break;
    case (TorsoStateDesired::TargetPoint::WBCOM): {
      feedforwardTermInControlFrame = totalMass * torso_.getDesiredState().getLinearAccelerationTargetInControlFrame().vector();
    } break;
    case (TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ): {
      feedforwardTermInControlFrame.x() = totalMass * torso_.getDesiredState().getLinearAccelerationTargetInControlFrame().x();
      feedforwardTermInControlFrame.y() = totalMass * torso_.getDesiredState().getLinearAccelerationTargetInControlFrame().y();
      feedforwardTermInControlFrame.z() =
          torso_.getProperties().getMass() * torso_.getDesiredState().getLinearAccelerationTargetInControlFrame().z();
    } break;
    default:
      MELO_ERROR_STREAM("VMC: Target Point is not valid: " << static_cast<unsigned int>(torso_.getDesiredState().getTargetPoint()));
      return false;
  }

  if (isIntegratingTheErrors_) {
    virtualIntegralForceInControlFrame_ +=
        Force(dt * integralGainTranslation_.getValue().cwiseProduct(positionErrorInControlFrame_.toImplementation()));

    for (int i = 0; i < virtualIntegralForceInControlFrame_.toImplementation().size(); i++) {
      virtualIntegralForceInControlFrame_(i) =
          robot_utils::boundToRange(virtualIntegralForceInControlFrame_(i), -virtualIntegralForceLimit_(i), virtualIntegralForceLimit_(i));
    }
  }

  virtualForceInBaseFrame_ =
      orientationControlToBase.rotate(
          Force(proportionalGainTranslation_.getValue().cwiseProduct(positionErrorInControlFrame_.toImplementation()))) +
      orientationControlToBase.rotate(
          Force(derivativeGainTranslation_.getValue().cwiseProduct(linearVelocityErrorInControlFrame_.toImplementation()))) +
      orientationControlToBase.rotate(virtualIntegralForceInControlFrame_) +
      orientationControlToBase.rotate(Force(feedforwardGainTranslation_.getValue().cwiseProduct(feedforwardTermInControlFrame))) +
      gravityCompensationForceInBaseFrame_ + offsetForceInBaseFrame_;

  return true;
}

bool VirtualModelController::computeVirtualTorque(double dt) {
  const RotationQuaternion& orientationControlToBase = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase();

  const Eigen::Vector3d feedforwardTermInControlFrame(0.0, 0.0, torso_.getDesiredState().getAngularVelocityBaseInControlFrame().z());

  if (isIntegratingTheErrors_) {
    virtualIntegralTorqueInBaseFrame_ += Torque(dt * integralGainRotation_.getValue().cwiseProduct(orientationError_));

    int virtualIntegralTorqueDimension = loco::Torque::Dimension;
    for (int i = 0; i < virtualIntegralTorqueDimension; i++) {
      virtualIntegralTorqueInBaseFrame_(i) =
          robot_utils::boundToRange(virtualIntegralTorqueInBaseFrame_(i), -virtualIntegralTorqueLimit_(i), virtualIntegralTorqueLimit_(i));
    }
  }

  virtualTorqueInBaseFrame_ =
      Torque(proportionalGainRotation_.getValue().cwiseProduct(orientationError_)) +
      orientationControlToBase.rotate(
          Torque(derivativeGainRotation_.getValue().cwiseProduct(angularVelocityErrorInControlFrame_.toImplementation()))) +
      virtualIntegralTorqueInBaseFrame_ +
      orientationControlToBase.rotate(Torque(feedforwardGainRotation_.getValue().cwiseProduct(feedforwardTermInControlFrame))) +
      gravityCompensationTorqueInBaseFrame_ + offsetTorqueInBaseFrame_;

  //  std::cout << "--------------------" << std::endl
  //      << "ornt err: " << Torque(proportionalGainRotation_.cwiseProduct(orientationError_)) << std::endl
  //      << "derivative err: " <<
  //      orientationControlToBase.rotate(Torque(derivativeGainRotation_.cwiseProduct(angularVelocityErrorInControlFrame_.toImplementation())))
  //      << std::endl
  //      << "ff: " << orientationControlToBase.rotate(Torque(feedforwardGainRotation_.cwiseProduct(feedforwardTermInControlFrame))) <<
  //      std::endl
  //      << "grav comp: " << gravityCompensationTorqueInBaseFrame_ << std::endl;

  return true;
}

void VirtualModelController::print(std::ostream& out) const {
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
  std::cout.precision(3);

  kindr::EulerAnglesYprPD errorYawRollPitch(orientationError_);
  Force netForce, netForceError;
  Torque netTorque, netTorqueError;
  contactForceDistribution_.getNetForceAndTorqueOnBase(netForce, netTorque);
  netForceError = virtualForceInBaseFrame_ - netForce;
  netTorqueError = virtualTorqueInBaseFrame_ - netTorque;

  out << "Position error: " << positionErrorInControlFrame_.toImplementation().format(CommaInitFmt) << std::endl;
  out << "Orientation error: " << orientationError_.format(CommaInitFmt) << std::endl;
  out << "Linear velocity error: " << linearVelocityErrorInControlFrame_.toImplementation().format(CommaInitFmt) << std::endl;
  out << "Angular velocity error: " << angularVelocityErrorInControlFrame_.toImplementation().format(CommaInitFmt) << std::endl;
  out << "----------------------------------------" << std::endl;
  out << "Virtual gravity compensation force: " << gravityCompensationForceInBaseFrame_.toImplementation().format(CommaInitFmt)
      << std::endl;
  out << "Virtual gravity compensation torque: " << gravityCompensationTorqueInBaseFrame_.toImplementation().format(CommaInitFmt)
      << std::endl;
  out << "Virtual integral force: " << virtualIntegralForceInControlFrame_ << std::endl;
  out << "Virtual integral torque: " << virtualIntegralTorqueInBaseFrame_ << std::endl;
  out << "Virtual offset force: " << offsetForceInBaseFrame_ << std::endl;
  out << "Virtual offset torque: " << offsetTorqueInBaseFrame_ << std::endl;
  out << "----------------------------------------" << std::endl;
  out << "Desired (cumulated) virtual force: " << virtualForceInBaseFrame_.toImplementation().format(CommaInitFmt) << std::endl;
  out << "Desired (cumulated) virtual torque:" << virtualTorqueInBaseFrame_.toImplementation().format(CommaInitFmt) << std::endl;
  out << "----------------------------------------" << std::endl;
  out << "Net force error" << netForceError.toImplementation().format(CommaInitFmt) << std::endl;
  out << "Net torque error" << netTorqueError.toImplementation().format(CommaInitFmt) << std::endl;
  out << "----------------------------------------" << std::endl;
  out << "Gravity compensation percentage: " << gravityCompensationForcePercentage_ << std::endl;
  out << "Is " << (isIntegratingTheErrors_ ? "" : "not ") << "integrating errors." << std::endl;
  out << "Virtual integral force limits: " << virtualIntegralForceLimit_ << std::endl;
  out << "Virtual integral torque limits: " << virtualIntegralTorqueLimit_ << std::endl;
  out << "Proportional gain translation: " << proportionalGainTranslation_.getValue().transpose() << std::endl;
  out << "Integral gain translation: " << integralGainTranslation_.getValue().transpose() << std::endl;
  out << "Derivative gain translation: " << derivativeGainTranslation_.getValue().transpose() << std::endl;
  out << "Feedforward gain translation: " << feedforwardGainTranslation_.getValue().transpose() << std::endl;
  out << "Proportional gain rotation: " << proportionalGainRotation_.getValue().transpose() << std::endl;
  out << "Integral gain rotation: " << integralGainRotation_.getValue().transpose() << std::endl;
  out << "Derivative gain rotation: " << derivativeGainRotation_.getValue().transpose() << std::endl;
  out << "Feedforward gain rotation: " << integralGainRotation_.getValue().transpose();
}

const Force& VirtualModelController::getDesiredVirtualForceInBaseFrame() const {
  return virtualForceInBaseFrame_;
}

const Torque& VirtualModelController::getDesiredVirtualTorqueInBaseFrame() const {
  return virtualTorqueInBaseFrame_;
}

void VirtualModelController::getDistributedVirtualForceAndTorqueInBaseFrame(Force& netForce, Torque& netTorque) const {
  contactForceDistribution_.getNetForceAndTorqueOnBase(netForce, netTorque);
}

bool VirtualModelController::loadParameters(const TiXmlHandle& handle) {
  resetIntegrator();
  offsetForceInBaseFrame_.setZero();
  offsetTorqueInBaseFrame_.setZero();

  // Load contact force distribution parameters
  contactForceDistribution_.loadParameters(handle);

  TiXmlHandle vmcHandle = handle;
  if (!tinyxml_tools::getChildHandle(vmcHandle, handle, "VirtualModelController")) {
    return false;
  }

  TiXmlHandle gainsHandle = handle;
  if (!tinyxml_tools::getChildHandle(gainsHandle, vmcHandle, "Gains")) {
    return false;
  }

  TiXmlHandle headingHandle = handle;
  if (!tinyxml_tools::getChildHandle(headingHandle, gainsHandle, "Heading")) {
    return false;
  }

  TiXmlHandle lateralHandle = handle;
  if (!tinyxml_tools::getChildHandle(lateralHandle, gainsHandle, "Lateral")) {
    return false;
  }

  TiXmlHandle verticalHandle = handle;
  if (!tinyxml_tools::getChildHandle(verticalHandle, gainsHandle, "Vertical")) {
    return false;
  }

  TiXmlHandle rollHandle = handle;
  if (!tinyxml_tools::getChildHandle(rollHandle, gainsHandle, "Roll")) {
    return false;
  }

  TiXmlHandle pitchHandle = handle;
  if (!tinyxml_tools::getChildHandle(pitchHandle, gainsHandle, "Pitch")) {
    return false;
  }

  TiXmlHandle yawHandle = handle;
  if (!tinyxml_tools::getChildHandle(yawHandle, gainsHandle, "Yaw")) {
    return false;
  }

  Eigen::Vector3d vector;
  tinyxml_tools::loadParameter(vector.x(), headingHandle, "kp");
  tinyxml_tools::loadParameter(vector.y(), lateralHandle, "kp");
  tinyxml_tools::loadParameter(vector.z(), verticalHandle, "kp");
  proportionalGainTranslation_.setDefaultValue(vector);
  proportionalGainTranslation_.setValue(vector);

  tinyxml_tools::loadParameter(vector.x(), headingHandle, "kd");
  tinyxml_tools::loadParameter(vector.y(), lateralHandle, "kd");
  tinyxml_tools::loadParameter(vector.z(), verticalHandle, "kd");
  derivativeGainTranslation_.setDefaultValue(vector);
  derivativeGainTranslation_.setValue(vector);

  tinyxml_tools::loadParameter(vector.x(), headingHandle, "ki");
  tinyxml_tools::loadParameter(vector.y(), lateralHandle, "ki");
  tinyxml_tools::loadParameter(vector.z(), verticalHandle, "ki");
  integralGainTranslation_.setDefaultValue(vector);
  integralGainTranslation_.setValue(vector);

  tinyxml_tools::loadParameter(virtualIntegralForceLimit_.x(), headingHandle, "maxI");
  tinyxml_tools::loadParameter(virtualIntegralForceLimit_.y(), lateralHandle, "maxI");
  tinyxml_tools::loadParameter(virtualIntegralForceLimit_.z(), verticalHandle, "maxI");

  tinyxml_tools::loadParameter(vector.x(), headingHandle, "kff");
  tinyxml_tools::loadParameter(vector.y(), lateralHandle, "kff");
  tinyxml_tools::loadParameter(vector.z(), verticalHandle, "kff");
  feedforwardGainTranslation_.setDefaultValue(vector);
  feedforwardGainTranslation_.setValue(vector);

  tinyxml_tools::loadParameter(offsetForceInBaseFrame_.x(), headingHandle, "offset");
  tinyxml_tools::loadParameter(offsetForceInBaseFrame_.y(), lateralHandle, "offset");
  tinyxml_tools::loadParameter(offsetForceInBaseFrame_.z(), verticalHandle, "offset");

  tinyxml_tools::loadParameter(vector.x(), rollHandle, "kp");
  tinyxml_tools::loadParameter(vector.y(), pitchHandle, "kp");
  tinyxml_tools::loadParameter(vector.z(), yawHandle, "kp");
  proportionalGainRotation_.setDefaultValue(vector);
  proportionalGainRotation_.setValue(vector);

  tinyxml_tools::loadParameter(vector.x(), rollHandle, "kd");
  tinyxml_tools::loadParameter(vector.y(), pitchHandle, "kd");
  tinyxml_tools::loadParameter(vector.z(), yawHandle, "kd");
  derivativeGainRotation_.setDefaultValue(vector);
  derivativeGainRotation_.setValue(vector);

  tinyxml_tools::loadParameter(vector.x(), rollHandle, "ki");
  tinyxml_tools::loadParameter(vector.y(), pitchHandle, "ki");
  tinyxml_tools::loadParameter(vector.z(), yawHandle, "ki");
  integralGainRotation_.setDefaultValue(vector);
  integralGainRotation_.setValue(vector);

  tinyxml_tools::loadParameter(virtualIntegralTorqueLimit_.x(), rollHandle, "maxI");
  tinyxml_tools::loadParameter(virtualIntegralTorqueLimit_.y(), pitchHandle, "maxI");
  tinyxml_tools::loadParameter(virtualIntegralTorqueLimit_.z(), yawHandle, "maxI");

  tinyxml_tools::loadParameter(vector.x(), rollHandle, "kff");
  tinyxml_tools::loadParameter(vector.y(), pitchHandle, "kff");
  tinyxml_tools::loadParameter(vector.z(), yawHandle, "kff");
  feedforwardGainRotation_.setDefaultValue(vector);
  feedforwardGainRotation_.setValue(vector);

  tinyxml_tools::loadParameter(offsetTorqueInBaseFrame_.x(), rollHandle, "offset");
  tinyxml_tools::loadParameter(offsetTorqueInBaseFrame_.y(), pitchHandle, "offset");
  tinyxml_tools::loadParameter(offsetTorqueInBaseFrame_.z(), yawHandle, "offset");

  // Read the desired default control mode for support and non-support mode.
  const TiXmlHandle motionControlHandle = tinyxml_tools::getChildHandle(handle, "MotionController");
  const TiXmlHandle defaultModeHandle = tinyxml_tools::getChildHandle(motionControlHandle, "DefaultControlModeForLeg");
  std::string supportControlMode;
  std::string nonSupportControlMode;
  tinyxml_tools::loadParameter(supportControlMode, defaultModeHandle, "supportMode");
  tinyxml_tools::loadParameter(nonSupportControlMode, defaultModeHandle, "nonSupportMode");
  supportLegControlMode_ = topology_conversions::getControlModeEnumFromControlModeString(supportControlMode);
  nonSupportLegControlMode_ = topology_conversions::getControlModeEnumFromControlModeString(nonSupportControlMode);

  return true;
}

const Eigen::Vector3d& VirtualModelController::getProportionalGainTranslation() const {
  return proportionalGainTranslation_.getValue();
}

const Eigen::Vector3d& VirtualModelController::getDerivativeGainTranslation() const {
  return derivativeGainTranslation_.getValue();
}

const Eigen::Vector3d& VirtualModelController::getIntegralGainTranslation() const {
  return integralGainTranslation_.getValue();
}

const Eigen::Vector3d& VirtualModelController::getFeedforwardGainTranslation() const {
  return feedforwardGainTranslation_.getValue();
}

const Eigen::Vector3d& VirtualModelController::getProportionalGainRotation() const {
  return proportionalGainRotation_.getValue();
}

const Eigen::Vector3d& VirtualModelController::getDerivativeGainRotation() const {
  return derivativeGainRotation_.getValue();
}

const Eigen::Vector3d& VirtualModelController::getIntegralGainRotation() const {
  return integralGainRotation_.getValue();
}

const Eigen::Vector3d& VirtualModelController::getFeedforwardGainRotation() const {
  return feedforwardGainRotation_.getValue();
}

void VirtualModelController::setProportionalGainRotation(const Eigen::Vector3d& gains) {
  proportionalGainRotation_.setValue(gains);
}

void VirtualModelController::setDerivativeGainRotation(const Eigen::Vector3d& gains) {
  derivativeGainRotation_.setValue(gains);
}

void VirtualModelController::setFeedforwardGainRotation(const Eigen::Vector3d& gains) {
  feedforwardGainRotation_.setValue(gains);
}

void VirtualModelController::setIntegralGainsRotation(const Eigen::Vector3d& gains) {
  integralGainRotation_.setValue(gains);
}

void VirtualModelController::setProportionalGainTranslation(const Eigen::Vector3d& gains) {
  proportionalGainTranslation_.setValue(gains);
}

void VirtualModelController::setDerivativeGainTranslation(const Eigen::Vector3d& gains) {
  derivativeGainTranslation_.setValue(gains);
}

void VirtualModelController::setFeedforwardGainTranslation(const Eigen::Vector3d& gains) {
  feedforwardGainTranslation_.setValue(gains);
}

void VirtualModelController::setIntegralGainsTranslation(const Eigen::Vector3d& gains) {
  integralGainTranslation_.setValue(gains);
}

void VirtualModelController::setGainsHeading(double kp, double kd, double ki, double kff) {
  proportionalGainTranslation_.setValue(kp, 0);
  derivativeGainTranslation_.setValue(kd, 0);
  integralGainTranslation_.setValue(ki, 0);
  feedforwardGainTranslation_.setValue(kff, 0);
}
void VirtualModelController::setGainsLateral(double kp, double kd, double ki, double kff) {
  proportionalGainTranslation_.setValue(kp, 1);
  derivativeGainTranslation_.setValue(kd, 1);
  integralGainTranslation_.setValue(ki, 1);
  feedforwardGainTranslation_.setValue(kff, 1);
}
void VirtualModelController::setGainsVertical(double kp, double kd, double ki, double kff) {
  proportionalGainTranslation_.setValue(kp, 2);
  derivativeGainTranslation_.setValue(kd, 2);
  integralGainTranslation_.setValue(ki, 2);
  feedforwardGainTranslation_.setValue(kff, 2);
}
void VirtualModelController::setGainsRoll(double kp, double kd, double ki, double kff) {
  proportionalGainRotation_.setValue(kp, 0);
  derivativeGainRotation_.setValue(kd, 0);
  integralGainRotation_.setValue(ki, 0);
  feedforwardGainRotation_.setValue(kff, 0);
}

void VirtualModelController::setGainsPitch(double kp, double kd, double ki, double kff) {
  proportionalGainRotation_.setValue(kp, 1);
  derivativeGainRotation_.setValue(kd, 1);
  integralGainRotation_.setValue(ki, 1);
  feedforwardGainRotation_.setValue(kff, 1);
}
void VirtualModelController::setGainsYaw(double kp, double kd, double ki, double kff) {
  proportionalGainRotation_.setValue(kp, 2);
  derivativeGainRotation_.setValue(kd, 2);
  integralGainRotation_.setValue(ki, 2);
  feedforwardGainRotation_.setValue(kff, 2);
}

void VirtualModelController::getGainsHeading(double& kp, double& kd, double& ki, double& kff) {
  kp = proportionalGainTranslation_.getValue().x();
  kd = derivativeGainTranslation_.getValue().x();
  ki = integralGainTranslation_.getValue().x();
  kff = feedforwardGainTranslation_.getValue().x();
}

void VirtualModelController::getGainsLateral(double& kp, double& kd, double& ki, double& kff) {
  kp = proportionalGainTranslation_.getValue().y();
  kd = derivativeGainTranslation_.getValue().y();
  ki = integralGainTranslation_.getValue().y();
  kff = feedforwardGainTranslation_.getValue().y();
}

void VirtualModelController::getGainsVertical(double& kp, double& kd, double& ki, double& kff) {
  kp = proportionalGainTranslation_.getValue().z();
  kd = derivativeGainTranslation_.getValue().z();
  ki = integralGainTranslation_.getValue().z();
  kff = feedforwardGainTranslation_.getValue().z();
}

void VirtualModelController::getGainsRoll(double& kp, double& kd, double& ki, double& kff) {
  kp = proportionalGainRotation_.getValue().x();
  kd = derivativeGainRotation_.getValue().x();
  ki = integralGainRotation_.getValue().x();
  kff = feedforwardGainRotation_.getValue().x();
}

void VirtualModelController::getGainsPitch(double& kp, double& kd, double& ki, double& kff) {
  kp = proportionalGainRotation_.getValue().y();
  kd = derivativeGainRotation_.getValue().y();
  ki = integralGainRotation_.getValue().y();
  kff = feedforwardGainRotation_.getValue().y();
}

void VirtualModelController::getGainsYaw(double& kp, double& kd, double& ki, double& kff) {
  kp = proportionalGainRotation_.getValue().z();
  kd = derivativeGainRotation_.getValue().z();
  ki = integralGainRotation_.getValue().z();
  kff = feedforwardGainRotation_.getValue().z();
}

void VirtualModelController::setGravityCompensationForcePercentage(double percentage) {
  gravityCompensationForcePercentage_ = percentage;
}

double VirtualModelController::getGravityCompensationForcePercentage() const {
  return gravityCompensationForcePercentage_;
}

const ContactForceDistributionInterface& VirtualModelController::getContactForceDistribution() const {
  return contactForceDistribution_;
}

bool VirtualModelController::setToInterpolated(const MotionControllerBase& motionController1, const MotionControllerBase& motionController2,
                                               double t) {
  const auto& controller1 = dynamic_cast<const VirtualModelController&>(motionController1);
  const auto& controller2 = dynamic_cast<const VirtualModelController&>(motionController2);

  // Interpolate control gains.
  this->proportionalGainTranslation_.setValue(robot_utils::linearlyInterpolate(controller1.getProportionalGainTranslation(),
                                                                               controller2.getProportionalGainTranslation(), 0.0, 1.0, t));
  this->derivativeGainTranslation_.setValue(robot_utils::linearlyInterpolate(controller1.getDerivativeGainTranslation(),
                                                                             controller2.getDerivativeGainTranslation(), 0.0, 1.0, t));
  this->integralGainTranslation_.setValue(
      robot_utils::linearlyInterpolate(controller1.getIntegralGainTranslation(), controller2.getIntegralGainTranslation(), 0.0, 1.0, t));
  this->feedforwardGainTranslation_.setValue(robot_utils::linearlyInterpolate(controller1.getFeedforwardGainTranslation(),
                                                                              controller2.getFeedforwardGainTranslation(), 0.0, 1.0, t));
  this->proportionalGainRotation_.setValue(
      robot_utils::linearlyInterpolate(controller1.getProportionalGainRotation(), controller2.getProportionalGainRotation(), 0.0, 1.0, t));
  this->derivativeGainRotation_.setValue(
      robot_utils::linearlyInterpolate(controller1.getDerivativeGainRotation(), controller2.getDerivativeGainRotation(), 0.0, 1.0, t));
  this->integralGainRotation_.setValue(
      robot_utils::linearlyInterpolate(controller1.getIntegralGainRotation(), controller2.getIntegralGainRotation(), 0.0, 1.0, t));
  this->feedforwardGainRotation_.setValue(
      robot_utils::linearlyInterpolate(controller1.getFeedforwardGainRotation(), controller2.getFeedforwardGainRotation(), 0.0, 1.0, t));

  // Interpolate the contact force distribution objects.
  if (!contactForceDistribution_.setToInterpolated(controller1.getContactForceDistribution(), controller2.getContactForceDistribution(),
                                                   t)) {
    return false;
  }

  // Set the leg control modes.
  this->supportLegControlMode_ = controller2.getSupportLegControlMode();
  this->nonSupportLegControlMode_ = controller2.getNonSupportLegControlMode();

  return true;
}

const Eigen::Vector3d& VirtualModelController::getIntegralGainTranslation() {
  return integralGainTranslation_.getValue();
}

const Eigen::Vector3d& VirtualModelController::getIntegralGainRotation() {
  return integralGainRotation_.getValue();
}

const Force& VirtualModelController::getVirtualIntegralForceLimit() {
  return virtualIntegralForceLimit_;
}

const Torque& VirtualModelController::getVirtualIntegralTorqueLimit() {
  return virtualIntegralTorqueLimit_;
}

void VirtualModelController::resetIntegrator() {
  virtualIntegralForceInControlFrame_.setZero();
  virtualIntegralTorqueInBaseFrame_.setZero();
}

void VirtualModelController::setIntegralGainsToZero() {
  integralGainTranslation_.setValue(Eigen::Vector3d::Zero());
  integralGainRotation_.setValue(Eigen::Vector3d::Zero());
}

void VirtualModelController::setIsIntegratingTheErrors(bool isIntegratingTheErrors) {
  isIntegratingTheErrors_ = isIntegratingTheErrors;
}

const ControlMode& VirtualModelController::getSupportLegControlMode() const {
  return supportLegControlMode_;
}

const ControlMode& VirtualModelController::getNonSupportLegControlMode() const {
  return nonSupportLegControlMode_;
}

} /* namespace loco */
