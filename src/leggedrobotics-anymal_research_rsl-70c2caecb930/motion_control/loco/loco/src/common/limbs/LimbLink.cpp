/*
 * LimbLink.cpp
 *
 *  Created on: May 18, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

// loco
#include "loco/common/limbs/LimbLink.hpp"

namespace loco {

LimbLink::LimbLink(int numDofs)
    : linkId_(0u), mass_(0.0), positionBaseToCoMInBaseFrame_(), jacobianBaseToCoMInBaseFrame_(TranslationJacobian::Zero(3, numDofs)) {}

void LimbLink::setLinkId(const unsigned int linkId) {
  linkId_ = linkId;
}

unsigned int LimbLink::getLinkId() const {
  return linkId_;
}

void LimbLink::setMass(double mass) {
  mass_ = mass;
}

double LimbLink::getMass() const {
  return mass_;
}

void LimbLink::setTranslationJacobianBaseToCoMInBaseFrame(const TranslationJacobian& jacobian) {
  jacobianBaseToCoMInBaseFrame_ = jacobian;
}

const TranslationJacobian& LimbLink::getTranslationJacobianBaseToCoMInBaseFrame() const {
  return jacobianBaseToCoMInBaseFrame_;
}

void LimbLink::setBaseToCoMPositionInBaseFrame(const Position& position) {
  positionBaseToCoMInBaseFrame_ = position;
}

const Position& LimbLink::getBaseToCoMPositionInBaseFrame() const {
  return positionBaseToCoMInBaseFrame_;
}

void LimbLink::setBaseToLinkPositionInBaseFrame(const Position& position) {
  positionBaseToLinkInBaseFrame_ = position;
}

const Position& LimbLink::getBaseToLinkPositionInBaseFrame() const {
  return positionBaseToLinkInBaseFrame_;
}

} /* namespace loco */
