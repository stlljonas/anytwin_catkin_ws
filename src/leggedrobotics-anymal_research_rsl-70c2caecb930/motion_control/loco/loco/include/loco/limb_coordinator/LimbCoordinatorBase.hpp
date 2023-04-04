/*!
 * @file     LimbCoordinatorBase.hpp
 * @author   Christian Gehring
 * @date     Feb, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/gait_pattern/GaitPatternBase.hpp"

// tinyxml
#include <tinyxml.h>

namespace loco {

class LimbCoordinatorBase : public ModuleBase {
 public:
  LimbCoordinatorBase() = default;
  ~LimbCoordinatorBase() override = default;

  bool initialize(double dt) override = 0;
  /*! Advance in time
   * @param dt  time step [s]
   */
  bool advance(double dt) override = 0;

  virtual GaitPatternBase* getGaitPatternPtr() = 0;

  bool loadParameters(const TiXmlHandle& handle) override = 0;

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  If t is 0, the current setting is set to limbCoordinator1, 1 -> limbCoordinator2, and values in between
   *  correspond to interpolated parameter set.
   * @param limbCoordinator1
   * @param limbCoordinator2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const LimbCoordinatorBase& limbCoordinator1, const LimbCoordinatorBase& limbCoordinator2, double /*t*/);
};

} /* namespace loco */
