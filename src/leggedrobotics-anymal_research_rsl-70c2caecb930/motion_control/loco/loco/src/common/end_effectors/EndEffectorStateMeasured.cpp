/*
 * EndEffectorStateMeasured.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

// loco
#include <loco/common/end_effectors/EndEffectorStateMeasured.hpp>

namespace loco {

EndEffectorStateMeasured::EndEffectorStateMeasured()
    : EndEffectorStateBase(),
      measuredJointStatesLimb_(nullptr),
      translationJacobianBaseToEndEffectorInBaseFrame_(),
      translationJacobianWorldToEndEffectorInWorldFrame_(),
      rotationJacobianBaseToEndEffectorInBaseFrame_(),
      rotationJacobianWorldToEndEffectorInWorldFrame_(),
      translationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame_(),
      translationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame_(),
      rotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame_(),
      rotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame_() {}

void EndEffectorStateMeasured::setMeasuredJointStatesLimb(MeasuredJointStates* const measuredJointStatesLimb) {
  measuredJointStatesLimb_ = measuredJointStatesLimb;
}

void EndEffectorStateMeasured::setPositionBaseToEndEffectorInBaseFrame(const Position& positionBaseToEndEffectorInBaseFrame) {
  positionBaseToEndEffectorInBaseFrame_ = positionBaseToEndEffectorInBaseFrame;
}

const Position& EndEffectorStateMeasured::getPositionBaseToEndEffectorInBaseFrame() const {
  return positionBaseToEndEffectorInBaseFrame_;
}

const TranslationJacobianLimb& EndEffectorStateMeasured::getTranslationJacobianBaseToEndEffectorInBaseFrame() const {
  return translationJacobianBaseToEndEffectorInBaseFrame_;
}

void EndEffectorStateMeasured::setTranslationJacobianBaseToEndEffectorInBaseFrame(
    const TranslationJacobianLimb& translationJacobianBaseToEndEffectorInBaseFrame) {
  translationJacobianBaseToEndEffectorInBaseFrame_ = translationJacobianBaseToEndEffectorInBaseFrame;
}

const TranslationJacobian& EndEffectorStateMeasured::getTranslationJacobianWorldToEndEffectorInWorldFrame() const {
  return translationJacobianWorldToEndEffectorInWorldFrame_;
}

void EndEffectorStateMeasured::setTranslationJacobianWorldToEndEffectorInWorldFrame(
    const TranslationJacobian& translationJacobianWorldToEndEffectorInWorldFrame) {
  translationJacobianWorldToEndEffectorInWorldFrame_ = translationJacobianWorldToEndEffectorInWorldFrame;
}

const RotationJacobianLimb& EndEffectorStateMeasured::getRotationJacobianBaseToEndEffectorInBaseFrame() const {
  return rotationJacobianBaseToEndEffectorInBaseFrame_;
}

void EndEffectorStateMeasured::setRotationJacobianBaseToEndEffectorInBaseFrame(
    const RotationJacobianLimb& rotationJacobianBaseToEndEffectorInBaseFrame) {
  rotationJacobianBaseToEndEffectorInBaseFrame_ = rotationJacobianBaseToEndEffectorInBaseFrame;
}

const RotationJacobian& EndEffectorStateMeasured::getRotationJacobianWorldToEndEffectorInWorldFrame() const {
  return rotationJacobianWorldToEndEffectorInWorldFrame_;
}

void EndEffectorStateMeasured::setRotationJacobianWorldToEndEffectorInWorldFrame(
    const RotationJacobian& rotationJacobianWorldToEndEffectorInWorldFrame) {
  rotationJacobianWorldToEndEffectorInWorldFrame_ = rotationJacobianWorldToEndEffectorInWorldFrame;
}

const TranslationJacobianLimb& EndEffectorStateMeasured::getTranslationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame() const {
  return translationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame_;
}

void EndEffectorStateMeasured::setTranslationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame(
    const TranslationJacobianLimb& translationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame) {
  translationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame_ = translationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame;
}

const TranslationJacobian& EndEffectorStateMeasured::getTranslationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame() const {
  return translationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame_;
}

void EndEffectorStateMeasured::setTranslationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame(
    const TranslationJacobian& translationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame) {
  translationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame_ = translationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame;
}

const RotationJacobianLimb& EndEffectorStateMeasured::getRotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame() const {
  return rotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame_;
}

void EndEffectorStateMeasured::setRotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame(
    const RotationJacobianLimb& rotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame) {
  rotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame_ = rotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame;
}

const RotationJacobian& EndEffectorStateMeasured::getRotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame() const {
  return rotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame_;
}

void EndEffectorStateMeasured::setRotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame(
    const RotationJacobian& rotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame) {
  rotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame_ = rotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame;
}

} /* namespace loco */
