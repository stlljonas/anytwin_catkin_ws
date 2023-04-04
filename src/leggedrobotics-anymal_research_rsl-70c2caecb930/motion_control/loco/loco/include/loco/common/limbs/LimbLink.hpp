/*
 * LimbLink.hpp
 *
 *  Created on: May 18, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"

namespace loco {

class LimbLink {
 public:
  explicit LimbLink(int numDofs = -1);
  virtual ~LimbLink() = default;

  void setLinkId(const unsigned int linkId);
  unsigned int getLinkId() const;

  void setMass(double mass);
  double getMass() const;

  void setTranslationJacobianBaseToCoMInBaseFrame(const TranslationJacobian& jacobian);
  const TranslationJacobian& getTranslationJacobianBaseToCoMInBaseFrame() const;

  void setBaseToCoMPositionInBaseFrame(const Position& position);
  const Position& getBaseToCoMPositionInBaseFrame() const;

  void setBaseToLinkPositionInBaseFrame(const Position& position);
  const Position& getBaseToLinkPositionInBaseFrame() const;

 protected:
  unsigned int linkId_;
  double mass_;
  Position positionBaseToCoMInBaseFrame_;
  Position positionBaseToLinkInBaseFrame_;
  TranslationJacobian jacobianBaseToCoMInBaseFrame_;
};

} /* namespace loco */
