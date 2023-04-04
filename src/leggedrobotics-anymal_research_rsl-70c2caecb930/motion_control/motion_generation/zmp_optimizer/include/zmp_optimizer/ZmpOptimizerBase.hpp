/*
 * ZmpOptimizerBase.hpp
 *
 *  Created on: Nov 25, 2014
 *      Author: Dario Bellicoso
 */

#pragma once

// zmp optimizer
#include <zmp_optimizer/MotionPlan.hpp>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco {

class ZmpOptimizerBase {
 public:
  ZmpOptimizerBase() = default;
  virtual ~ZmpOptimizerBase() = default;

  virtual bool initialize() = 0;
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;
  virtual bool addToLogger() = 0;

  virtual bool computeTrajectory(zmp::MotionPlan& motionPlan) = 0;

  virtual void stop() {}
};

} /* namespace loco */
