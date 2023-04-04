/*
 * MotionPlanAngular.cpp
 *
 *  Created on: 07.08, 2018
 *      Author: Fabian Jenelten
 */

// motion generation
#include "motion_generation_utils/MotionPlanAngular.hpp"


namespace motion_generation {

MotionPlanAngular::MotionPlanAngular()
    : MotionPlanBase() {
}

EulerAnglesZyx MotionPlanAngular::getAnglesZyxInitialBaseToPlane() const {
  return {
      initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::yaw],
      initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::pitch],
      initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::roll]};
}

EulerAnglesZyxDiff MotionPlanAngular::getInitialAngularRatesZyxInPlaneFrame() const {
  return {
      initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::yaw],
      initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::pitch],
      initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::roll]};
}

EulerAnglesZyxDiff MotionPlanAngular::getInitialAngularAccelerationZyxInPlaneFrame() const {
  return {
      initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::yaw],
      initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::pitch],
      initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::roll]};
}

EulerAnglesZyx MotionPlanAngular::getAnglesZyxFinalBaseToPlane() const {
  return {
      finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::yaw],
      finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::pitch],
      finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::roll]};
}

EulerAnglesZyxDiff MotionPlanAngular::getFinalAngularRatesZyxInPlaneFrame() const {
  return {
      finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::yaw],
      finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::pitch],
      finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::roll]};
}

EulerAnglesZyxDiff MotionPlanAngular::getFinalAngularAccelerationZyxInPlaneFrame() const {
  return {
      finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::yaw],
      finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::pitch],
      finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::roll]};
}

void MotionPlanAngular::setAnglesZyxInitialBaseToPlane(const EulerAnglesZyx& initialStateInPlaneFrame) {
  initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::yaw]   = initialStateInPlaneFrame.yaw();
  initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::pitch] = initialStateInPlaneFrame.pitch();
  initialRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::roll]  = initialStateInPlaneFrame.roll();
}

void MotionPlanAngular::setInitialAngularRatesZyxInPlaneFrame(const EulerAnglesZyxDiff& initialStateInPlaneFrame) {
  initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::yaw]   = initialStateInPlaneFrame.yaw();
  initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::pitch] = initialStateInPlaneFrame.pitch();
  initialRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::roll]  = initialStateInPlaneFrame.roll();
}

void MotionPlanAngular::setInitialAngularAccelerationZyxInPlaneFrame(const EulerAnglesZyxDiff& initialStateInPlaneFrame) {
  initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::yaw]   = initialStateInPlaneFrame.yaw();
  initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::pitch] = initialStateInPlaneFrame.pitch();
  initialRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::roll]  = initialStateInPlaneFrame.roll();
}

void MotionPlanAngular::setAnglesZyxFinalBaseToPlane(const EulerAnglesZyx& finalStateInPlaneFrame) {
  finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::yaw]   = finalStateInPlaneFrame.yaw();
  finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::pitch] = finalStateInPlaneFrame.pitch();
  finalRobotStateInPlaneFrame_[zmp::Derivative::Zero][zmp::CogDim::roll]  = finalStateInPlaneFrame.roll();
}

void MotionPlanAngular::setFinalAngularRatesZyxInPlaneFrame(const EulerAnglesZyxDiff& finalStateInPlaneFrame) {
  finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::yaw]   = finalStateInPlaneFrame.yaw();
  finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::pitch] = finalStateInPlaneFrame.pitch();
  finalRobotStateInPlaneFrame_[zmp::Derivative::First][zmp::CogDim::roll]  = finalStateInPlaneFrame.roll();
}

void MotionPlanAngular::setFinalAngularAccelerationZyxInPlaneFrame(const EulerAnglesZyxDiff& finalStateInPlaneFrame) {
  finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::yaw]   = finalStateInPlaneFrame.yaw();
  finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::pitch] = finalStateInPlaneFrame.pitch();
  finalRobotStateInPlaneFrame_[zmp::Derivative::Second][zmp::CogDim::roll]  = finalStateInPlaneFrame.roll();
}

} /* namespace motion_generation */
