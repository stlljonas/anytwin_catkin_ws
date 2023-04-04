/*
 * LimbBase.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/common/limbs/LimbBase.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

namespace loco {

LimbBase::LimbBase(const std::string name, const unsigned int numDofLimb, LimbPropertiesPtr&& properties, EndEffectorBasePtr&& endEffector)
    : ModuleBase(name),
      numDofLimb_(numDofLimb),
      limbProperties_(std::move(properties)),
      endEffector_(std::move(endEffector)),
      limbStateDesired_(new LimbStateDesired(numDofLimb)),
      limbStateMeasured_(new LimbStateMeasured(numDofLimb)),
      limbStrategy_(new loco::LimbStrategy()),
      links_(),
      jointNames_(),
      loadFactor_(1.0),
      frictionModulation_(1.0) {
  endEffector_->setJointStatesLimb(limbStateDesired_.get(), limbStateMeasured_.get());
}

LimbBase::LimbBase(const std::string name, const unsigned int numDofLimb, LimbPropertiesPtr&& properties, EndEffectorBasePtr&& endEffector,
                   LimbStateMeasuredPtr&& stateMeasured, LimbStateDesiredPtr&& stateDesired)
    : ModuleBase(name),
      numDofLimb_(numDofLimb),
      limbProperties_(std::move(properties)),
      endEffector_(std::move(endEffector)),
      limbStateMeasured_(std::move(stateMeasured)),
      limbStateDesired_(std::move(stateDesired)),
      limbStrategy_(new loco::LimbStrategy()),
      links_(),
      jointNames_(),
      loadFactor_(1.0),
      frictionModulation_(1.0) {
  endEffector_->setJointStatesLimb(limbStateDesired_.get(), limbStateMeasured_.get());
}

unsigned int LimbBase::getNumDofLimb() const {
  return numDofLimb_;
}

bool LimbBase::addVariablesToLog(const std::string& logNameSpace) const {
  // Log stance load factor.
  signal_logger::add(loadFactor_, "loadFactor", logNameSpace);
  return true;
}

JointPositions LimbBase::getInitializedJointPositions() const {
  return JointPositions::Zero(numDofLimb_);
}

JointVelocities LimbBase::getInitializedJointVelocities() const {
  return JointVelocities::Zero(numDofLimb_);
}

JointAccelerations LimbBase::getInitializedJointAccelerations() const {
  return JointAccelerations::Zero(numDofLimb_);
}

JointTorques LimbBase::getInitializedJointTorques() const {
  return JointTorques::Zero(numDofLimb_);
}

JointControlModes LimbBase::getInitializedJointControlModes() const {
  return JointControlModes::Zero(numDofLimb_);
}

TranslationJacobianLimb LimbBase::getInitializedTranslationJacobianLimb() const {
  return TranslationJacobianLimb::Zero(cartesian::forceVectorSize, numDofLimb_);
}

TranslationJacobian LimbBase::getInitializedTranslationJacobian() const {
  return TranslationJacobian::Zero(cartesian::forceVectorSize, getNumDofU());
}

RotationJacobianLimb LimbBase::getInitializedRotationJacobianLimb() const {
  return RotationJacobianLimb::Zero(cartesian::forceVectorSize, numDofLimb_);
}

RotationJacobian LimbBase::getInitializedRotationJacobian() const {
  return RotationJacobian::Zero(cartesian::forceVectorSize, getNumDofU());
}

JointNames LimbBase::getInitializedJointNames() const {
  return JointNames(numDofLimb_, "");
}

void LimbBase::populateJointPositions(JointPositions& joints) const {
  joints = JointPositions::Zero(numDofLimb_);
}

void LimbBase::populateJointVelocities(JointVelocities& jointVelocities) const {
  jointVelocities = JointVelocities::Zero(numDofLimb_);
}

void LimbBase::populateJointAccelerations(JointAccelerations& jointAccelerations) const {
  jointAccelerations = JointAccelerations::Zero(numDofLimb_);
}

void LimbBase::populateJointTorques(JointTorques& jointTorques) const {
  jointTorques = JointTorques::Zero(numDofLimb_);
}

void LimbBase::populateJointControlModes(JointControlModes& jointControlModes) const {
  jointControlModes = JointControlModes::Zero(numDofLimb_);
}

void LimbBase::populateTranslationJacobianLimb(TranslationJacobianLimb& translationJacobianLimb) const {
  translationJacobianLimb = TranslationJacobian::Zero(3, numDofLimb_);
}

void LimbBase::populateTranslationJacobian(TranslationJacobian& translationJacobian) const {
  translationJacobian = TranslationJacobian::Zero(3, getNumDofU());
}

void LimbBase::populateRotationJacobianLimb(RotationJacobianLimb& rotationJacobianLimb) const {
  rotationJacobianLimb = RotationJacobian::Zero(3, numDofLimb_);
}

void LimbBase::populateRotationJacobian(RotationJacobian& rotationJacobian) const {
  rotationJacobian = RotationJacobian::Zero(3, getNumDofU());
}

void LimbBase::populateSpatialJacobian(SpatialJacobian& spatialJacobian) const {
  spatialJacobian = SpatialJacobian::Zero(6, getNumDofU());
}

void LimbBase::populateJointNames(JointNames& jointNames) const {
  jointNames.resize(numDofLimb_, "");
}

const LimbStateDesired& LimbBase::getLimbStateDesired() const {
  return *limbStateDesired_;
}

LimbStateDesired* LimbBase::getLimbStateDesiredPtr() {
  return limbStateDesired_.get();
}

LimbStateMeasured* LimbBase::getLimbStateMeasuredPtr() {
  return limbStateMeasured_.get();
}

const LimbStateMeasured& LimbBase::getLimbStateMeasured() const {
  return *limbStateMeasured_;
}

EndEffectorBase* LimbBase::getEndEffectorPtr() {
  return endEffector_.get();
}

const EndEffectorBase& LimbBase::getEndEffector() const {
  return *endEffector_;
}

LimbProperties* LimbBase::getLimbPropertiesPtr() {
  return limbProperties_.get();
}

const LimbProperties& LimbBase::getLimbProperties() const {
  return *limbProperties_;
}

LimbStrategy* LimbBase::getLimbStrategyPtr() {
  return limbStrategy_.get();
}

const LimbStrategy& LimbBase::getLimbStrategy() const {
  return *limbStrategy_;
}

LimbLinkGroup* LimbBase::getLinksPtr() {
  return &links_;
}

const LimbLinkGroup& LimbBase::getLinks() const {
  return links_;
}

double LimbBase::getLoadFactor() const {
  return loadFactor_;
}

void LimbBase::setLoadFactor(double loadFactor) {
  loadFactor_ = loadFactor;
}

void LimbBase::setFrictionModulation(double frictionModulation) {
  frictionModulation_ = frictionModulation;
}

double LimbBase::getFrictionModulation() const {
  return frictionModulation_;
}

JointNames* LimbBase::getJointNamesPtr() {
  return &jointNames_;
}

const JointNames& LimbBase::getJointNames() const {
  return jointNames_;
}

} /* namespace loco */
