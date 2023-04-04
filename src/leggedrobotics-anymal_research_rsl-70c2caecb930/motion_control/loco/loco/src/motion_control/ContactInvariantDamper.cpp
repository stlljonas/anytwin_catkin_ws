/*
 * ContactInvariantDamper.cpp
 *
 *  Created on: Aug 15, 2016
 *      Author: Christian Gehring
 */

#include "loco/motion_control/ContactInvariantDamper.hpp"
#include <message_logger/message_logger.hpp>

namespace loco {

ContactInvariantDamper::ContactInvariantDamper(WholeBody& wholeBody)
    : MotionControllerBase(wholeBody), headingDampingGain_(6.0, 0.0, 100.0), lateralDampingGain_(6.0, 0.0, 100.0) {
  logData_.clear();
  for (auto leg : wholeBody_.getLegs()) {
    logData_[leg] = LogData();
    logData_[leg].dampingJointTorques_ = leg->getInitializedJointTorques();
    logData_[leg].dampingContactForceInControlFrame_ = Force();
  }
}

bool ContactInvariantDamper::loadParameters(const TiXmlHandle& handle) {
  TiXmlElement* element;
  double value = 0.0;

  TiXmlHandle gainsHandle(handle.FirstChild("ContactInvariantDamper").FirstChild("Gains"));
  element = gainsHandle.Element();
  if (element == nullptr) {
    MELO_WARN("[ContactInvariantDamper::loadParameters] Could not find ContactInvariantDamper::Gains.");
    return false;
  }

  // Linear Velocity
  element = gainsHandle.FirstChild("LinearVelocity").Element();

  if (element->QueryDoubleAttribute("kd_x", &value) != TIXML_SUCCESS) {
    MELO_WARN("[ContactInvariantDamper::loadParameters] Could not find WholeBodyControl:Gains:LinearVelocity:kd_x.");
    return false;
  }
  headingDampingGain_.setDefaultValue(value);
  headingDampingGain_.setValue(value);

  if (element->QueryDoubleAttribute("kd_y", &value) != TIXML_SUCCESS) {
    MELO_WARN("[ContactInvariantDamper::loadParameters] Could not find WholeBodyControl:Gains:LinearVelocity:kd_y.");
    return false;
  }
  lateralDampingGain_.setDefaultValue(value);
  lateralDampingGain_.setValue(value);

  return true;
}

bool ContactInvariantDamper::addVariablesToLog(const std::string& ns) const {
  std::string namesp = ns + std::string{"/loco/cid/"};
  for (auto& data : logData_) {
    signal_logger::add(data.second.dampingContactForceInControlFrame_,
                       std::string{"dampingContactForceInControlFrame_"} + data.first->getName(), namesp, "N");
    signal_logger::add(data.second.dampingJointTorques_, std::string{"dampingJointTorques_"} + data.first->getName(), namesp, "Nm");
  }
  return true;
}

bool ContactInvariantDamper::addParametersToHandler(const std::string& ns) {
  if (!parameter_handler::handler->addParam(ns + "/loco/ContactInvariantDamper/headingDampingGain", headingDampingGain_)) {
    return false;
  }
  if (!parameter_handler::handler->addParam(ns + "/loco/ContactInvariantDamper/lateralDampingGain", lateralDampingGain_)) {
    return false;
  }
  return true;
}

bool ContactInvariantDamper::initialize(double dt) {
  for (auto leg : wholeBody_.getLegs()) {
    logData_[leg].dampingJointTorques_ = leg->getInitializedJointTorques();
    logData_[leg].dampingContactForceInControlFrame_ = Force();
  }
  return true;
}

bool ContactInvariantDamper::advance(double dt) {
  const Eigen::Vector3d dampingGains(headingDampingGain_.getValue(), lateralDampingGain_.getValue(), 0.0);
  auto& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();

  for (auto leg : *wholeBody_.getLegsPtr()) {
    if (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::ContactInvariant ||
        leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::Support) {
      JointTorques jointTorques = leg->getInitializedJointTorques();
      const Eigen::Vector3d linearVelocityFootInControlFrame =
          orientationWorldToControl.rotate(leg->getFoot().getStateMeasured().getLinearVelocityEndEffectorInWorldFrame()).toImplementation();
      const Force forceAtFootInControlFrame(dampingGains.cwiseProduct(-linearVelocityFootInControlFrame));
      logData_[leg].dampingContactForceInControlFrame_ = forceAtFootInControlFrame;
      const Force forceAtFootInWorldFrame = orientationWorldToControl.inverseRotate(forceAtFootInControlFrame);
      computeJointTorquesFromForceAtFootInWorldFrame(jointTorques, leg, forceAtFootInWorldFrame);

      logData_[leg].dampingJointTorques_ = jointTorques;

      // MELO_INFO_THROTTLE_STREAM(0.5, "CID: " << leg->getName() <<  " leg: forceInControl: " << forceAtFootInControlFrame << " torques: "
      // << jointTorques);
      jointTorques += leg->getLimbStateDesired().getJointTorques();
      leg->getLimbStateDesiredPtr()->setJointTorques(jointTorques);
    }
  }

  return true;
}

void ContactInvariantDamper::computeJointTorquesFromForceAtFootInWorldFrame(JointTorques& jointTorques, LegBase* leg,
                                                                            const Force& desiredContactForceAtFootInWorldFrame) const {
  const TranslationJacobian& jacobian = leg->getFoot().getStateMeasured().getTranslationJacobianBaseToEndEffectorInBaseFrame();
  const Force contactForceInBaseFrame = torso_.getMeasuredState().getOrientationWorldToBase().rotate(desiredContactForceAtFootInWorldFrame);
  jointTorques = JointTorques(jacobian.transpose() * contactForceInBaseFrame.toImplementation());
}

} /* namespace loco */
