/*!
 * @file 	    ComSupportControl.cpp
 * @author 	  Christian Gehring, Stelian Coros
 * @date		  Jul 17, 2012
 * @version 	1.0
 * @ingroup
 * @brief
 */

#include "loco/torso_control/ComSupportControlDynamicGait.hpp"
#include "loco/common/TerrainModelFreePlane.hpp"
#include "robot_utils/math/math.hpp"

namespace loco {

ComSupportControlDynamicGait::ComSupportControlDynamicGait(Legs& legs) : ComSupportControlBase(legs) {}

bool ComSupportControlDynamicGait::initialize(double dt) {
  return true;
}

bool ComSupportControlDynamicGait::advance(double dt) {
  const int nLegs = legs_.size();
  double legWeights[nLegs];
  for (int i = 0; i < nLegs; i++) {
    legWeights[i] = 1.0;
  }

  double sumWeights = 0.0;
  int iLeg = 0;
  for (auto leg : legs_) {
    //    if ((((leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
    //        (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) )
    //        || leg->getContactSchedule().shouldBeGrounded() ) {

    if (leg->getContactSchedule().shouldBeGrounded()) {
      double t = 1.0 - robot_utils::mapTo01Range(std::max(leg->getContactSchedule().getStancePhase(), 0.0),
                                                 startShiftAwayFromLegAtStancePhase_, 1.0);
      t = robot_utils::linearlyInterpolate(minSwingLegWeight_, 1.0, 0.0, 1.0, t);
      legWeights[iLeg] = t;
    } else {
      double t =
          robot_utils::mapTo01Range(std::max(leg->getContactSchedule().getSwingPhase(), 0.0), startShiftTowardsLegAtSwingPhase_, 1.0);
      t = robot_utils::linearlyInterpolate(minSwingLegWeight_, 1.0, 0.0, 1.0, t);
      legWeights[iLeg] = t;
    }
    sumWeights += legWeights[iLeg];
    iLeg++;
  }

  Position comTarget;

  if (sumWeights != 0.0) {
    iLeg = 0;
    for (auto leg : legs_) {
      //	            tprintf("leg %d(%s): stanceMode: %lf, swingMode: %lf. Weight:%lf\n", i, leg->getName(),
      // leg->getStancePhase(),leg->getSwingPhase(), legWeights[i]);
      comTarget += leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() * legWeights[iLeg];
      iLeg++;
    }
    comTarget /= sumWeights;
  } else {
    for (auto leg : legs_) {
      comTarget += leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    }
    comTarget /= legs_.size();
  }

  positionWorldToDesiredCoMInWorldFrame_ = comTarget + Position(headingOffset_, lateralOffset_, 0.0);
  positionWorldToDesiredCoMInWorldFrame_.z() = 0.0;

  return true;
}

const Position& ComSupportControlDynamicGait::getPositionWorldToDesiredCoMInWorldFrame() const {
  return positionWorldToDesiredCoMInWorldFrame_;
}

bool ComSupportControlDynamicGait::setToInterpolated(const ComSupportControlBase& supportPolygon1,
                                                     const ComSupportControlBase& supportPolygon2, double t) {
  const auto& supportPolygon1_this = dynamic_cast<const ComSupportControlDynamicGait&>(supportPolygon1);
  const auto& supportPolygon2_this = dynamic_cast<const ComSupportControlDynamicGait&>(supportPolygon2);

  this->minSwingLegWeight_ =
      robot_utils::linearlyInterpolate(supportPolygon1.getMinSwingLegWeight(), supportPolygon2_this.getMinSwingLegWeight(), 0, 1, t);
  this->startShiftAwayFromLegAtStancePhase_ = robot_utils::linearlyInterpolate(
      supportPolygon1_this.getStartShiftAwayFromLegAtStancePhase(), supportPolygon2_this.getStartShiftAwayFromLegAtStancePhase(), 0, 1, t);
  this->startShiftTowardsLegAtSwingPhase_ = robot_utils::linearlyInterpolate(
      supportPolygon1_this.getStartShiftTowardsLegAtSwingPhase(), supportPolygon2_this.getStartShiftTowardsLegAtSwingPhase(), 0, 1, t);

  this->headingOffset_ =
      robot_utils::linearlyInterpolate(supportPolygon1_this.getHeadingOffset(), supportPolygon2_this.getHeadingOffset(), 0, 1, t);

  this->lateralOffset_ =
      robot_utils::linearlyInterpolate(supportPolygon1_this.getLateralOffset(), supportPolygon2_this.getLateralOffset(), 0, 1, t);

  return true;
}

}  // namespace loco
