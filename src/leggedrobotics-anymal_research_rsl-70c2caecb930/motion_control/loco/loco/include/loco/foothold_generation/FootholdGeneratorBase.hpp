/*
 * FootholdGeneratorBase.hpp
 *
 *  Created on: Feb 1, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// tinyxml
#include <tinyxml.h>

// loco
#include "loco/common/typedefs.hpp"

namespace loco {

/*!
 * A foothold generator computes the next desired foothold for each leg.
 *
 * Inputs:
 * - for each leg, ``leg->limbStateMeasured->getPositionWorldToLimbBaseInWorldFrame()``
 *
 * Outputs:
 * - for each leg, ``leg->foot->stateDesired->setPositionWorldToFootholdInWorldFrame()
 * - for each leg, ``leg->legProperties->setDesiredDefaultSteppingPositionHipToFootInControlFrame()``
 *
 * Advanced by: foot placement strategy.
 *
 */
class FootholdGeneratorBase {
 public:
  FootholdGeneratorBase();
  virtual ~FootholdGeneratorBase() = default;

  virtual bool loadParameters(const TiXmlHandle& handle) = 0;
  virtual bool addVariablesToLog() { return true; }
};

} /* namespace loco */
