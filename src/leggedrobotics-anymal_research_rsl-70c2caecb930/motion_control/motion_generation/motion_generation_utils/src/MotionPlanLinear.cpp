/*
 * MotionPlanLinear.cpp
 *
 *  Created on: 07.08, 2018
 *      Author: Fabian Jenelten
 */


// motion generation
#include <motion_generation_utils/MotionPlanLinear.hpp>


namespace motion_generation {

MotionPlanLinear::MotionPlanLinear()
    : MotionPlanBase()
{

}


Position MotionPlanLinear::getPlaneToInitialPositionInPlaneFrame() const {
  return Position(
      initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::x],
      initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::y],
      initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::z]);
}

LinearVelocity MotionPlanLinear::getInitialVelocityInPlaneFrame() const {
  return LinearVelocity(
      initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::x],
      initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::y],
      initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::z]);
}

LinearAcceleration MotionPlanLinear::getInitialAccelerationInPlaneFrame() const {
  return LinearAcceleration(
      initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::x],
      initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::y],
      initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::z]);
}


Position MotionPlanLinear::getPlaneToFinalPositionInPlaneFrame() const {
  return Position(
      finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::x],
      finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::y],
      finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::z]);
}

LinearVelocity MotionPlanLinear::getFinalVelocityInPlaneFrame() const {
  return LinearVelocity(
      finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::x],
      finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::y],
      finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::z]);
}

LinearAcceleration MotionPlanLinear::getFinalAccelerationInPlaneFrame() const {
  return LinearAcceleration(
      finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::x],
      finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::y],
      finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::z]);
}

void MotionPlanLinear::getWorldToInitialPositionInWorldFrame(Position& translationalStateInWorldFrame) const {
  translationalStateInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().transform(getPlaneToInitialPositionInPlaneFrame());
}

void MotionPlanLinear::getWorldToFinalPositionInWorldFrame(Position& translationalStateInWorldFrame) const {
  translationalStateInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().transform(getPlaneToFinalPositionInPlaneFrame());
}

void MotionPlanLinear::getInitialVelocityInWorldFrame(LinearVelocity& translationalStateInWorldFrame) const {
  translationalStateInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(getInitialVelocityInPlaneFrame());
}

void MotionPlanLinear::getFinalVelocityInWorldFrame(LinearVelocity& translationalStateInWorldFrame) const {
  translationalStateInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(getFinalVelocityInPlaneFrame());
}

void MotionPlanLinear::getInitialAccelerationInWorldFrame(LinearAcceleration& translationalStateInWorldFrame) const {
  translationalStateInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(getInitialAccelerationInPlaneFrame());
}

void MotionPlanLinear::getFinalAccelerationInWorldFrame(LinearAcceleration& translationalStateInWorldFrame) const {
  translationalStateInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(getFinalAccelerationInPlaneFrame());
}

void MotionPlanLinear::setPlaneToInitialPositionInPlaneFrame(const Position& initialStateInPlaneFrame) {
  initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::x] = initialStateInPlaneFrame.x();
  initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::y] = initialStateInPlaneFrame.y();
  initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::z] = initialStateInPlaneFrame.z();
}

void MotionPlanLinear::setPlaneToFinalPositionInPlaneFrame(const Position& finalStateInPlaneFrame) {
  finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::x] = finalStateInPlaneFrame.x();
  finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::y] = finalStateInPlaneFrame.y();
  finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::z] = finalStateInPlaneFrame.z();
}

void MotionPlanLinear::setInitialVelocityInPlaneFrame(const LinearVelocity& initialStateInPlaneFrame) {
  initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::x] = initialStateInPlaneFrame.x();
  initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::y] = initialStateInPlaneFrame.y();
  initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::z] = initialStateInPlaneFrame.z();
}

void MotionPlanLinear::setFinalVelocityInPlaneFrame(const LinearVelocity& finalStateInPlaneFrame) {
  finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::x] = finalStateInPlaneFrame.x();
  finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::y] = finalStateInPlaneFrame.y();
  finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::z] = finalStateInPlaneFrame.z();
}

void MotionPlanLinear::setInitialAccelerationInPlaneFrame(const LinearAcceleration& initialStateInPlaneFrame) {
  initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::x] = initialStateInPlaneFrame.x();
  initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::y] = initialStateInPlaneFrame.y();
  initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::z] = initialStateInPlaneFrame.z();
}

void MotionPlanLinear::setFinalAccelerationInPlaneFrame(const LinearAcceleration& finalStateInPlaneFrame) {
  finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::x] = finalStateInPlaneFrame.x();
  finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::y] = finalStateInPlaneFrame.y();
  finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::z] = finalStateInPlaneFrame.z();
}

} /* namespace motion_generation */
