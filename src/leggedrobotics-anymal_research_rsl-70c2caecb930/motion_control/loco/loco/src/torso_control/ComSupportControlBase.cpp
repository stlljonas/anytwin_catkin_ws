/*!
 * @file     ComSupportControlBase.cpp
 * @author   Christian Gehring, C. Dario Bellicoso
 * @date     Oct 7, 2014
 * @brief
 */

#include <Eigen/Dense>
#include <cstdio>

#include "robot_utils/math/math.hpp"

#include "loco/torso_control/ComSupportControlBase.hpp"

namespace loco {

ComSupportControlBase::ComSupportControlBase(Legs& legs) : legs_(legs) {
  minSwingLegWeight_ = 0.0;
  startShiftAwayFromLegAtStancePhase_ = 0.0;
  startShiftTowardsLegAtSwingPhase_ = 0.0;
  headingOffset_ = 0.0;
  lateralOffset_ = 0.0;

  positionWorldToDesiredCoMInWorldFrame_.setZero();
  linearVelocityDesiredBaseInWorldFrame_.setZero();
}

bool ComSupportControlBase::loadParameters(const TiXmlHandle& hParameterSet) {
  bool success = true;
  const TiXmlHandle comSupportHandle = tinyxml_tools::getChildHandle(hParameterSet, "ComSupportControl");

  TiXmlHandle offsetHandle = tinyxml_tools::getChildHandle(comSupportHandle, "ComOffset");
  success = tinyxml_tools::loadParameter(headingOffset_, offsetHandle, "heading", 0.0) && success;
  success = tinyxml_tools::loadParameter(lateralOffset_, offsetHandle, "lateral", 0.0) && success;

  TiXmlHandle weightHandle = tinyxml_tools::getChildHandle(comSupportHandle, "Weight");
  success = tinyxml_tools::loadParameter(minSwingLegWeight_, weightHandle, "minSwingLegWeight", 0.0) && success;

  TiXmlHandle timingHandle = tinyxml_tools::getChildHandle(comSupportHandle, "Timing");
  success =
      tinyxml_tools::loadParameter(startShiftAwayFromLegAtStancePhase_, timingHandle, "startShiftAwayFromLegAtStancePhase", 0.0) && success;
  success =
      tinyxml_tools::loadParameter(startShiftTowardsLegAtSwingPhase_, timingHandle, "startShiftTowardsLegAtSwingPhase", 0.0) && success;

  return success;
}

bool ComSupportControlBase::saveParameters(TiXmlHandle& hParameterSet) {
  return false;
}

double ComSupportControlBase::getMinSwingLegWeight() const {
  return minSwingLegWeight_;
}
double ComSupportControlBase::getStartShiftAwayFromLegAtStancePhase() const {
  return startShiftAwayFromLegAtStancePhase_;
}
double ComSupportControlBase::getStartShiftTowardsLegAtSwingPhase() const {
  return startShiftTowardsLegAtSwingPhase_;
}
double ComSupportControlBase::getLateralOffset() const {
  return lateralOffset_;
}
double ComSupportControlBase::getHeadingOffset() const {
  return headingOffset_;
}

const loco::LinearVelocity& ComSupportControlBase::getLinearVelocityDesiredBaseInWorldFrame() const {
  return linearVelocityDesiredBaseInWorldFrame_;
}

} /* namespace loco */
