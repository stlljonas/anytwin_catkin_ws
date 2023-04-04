/*!
 * @file     LocomotionControllerBase.hpp
 * @author   Christian Gehring
 * @date     Feb, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */
#pragma once

// loco
#include "loco/common/ModuleBase.hpp"

// STL
#include <string>

namespace loco {

class LocomotionControllerBase : public ModuleBase {
 public:
  LocomotionControllerBase();
  ~LocomotionControllerBase() override = default;

  //! If no action has to be taken between advancing measurements and setpoints use this function
  bool advance(double dt) override { return advanceMeasurements(dt) && advanceSetPoints(dt); }

  virtual bool advanceMeasurements(double dt) = 0;
  virtual bool advanceSetPoints(double dt) = 0;

  /*! @returns true if the controller is initialized.
   */
  virtual bool isInitialized() const { return isInitialized_; }

  /*! @returns the run time of the controller in seconds.
   */
  virtual double getRuntime() const { return runtime_; }

 protected:
  bool isInitialized_;
  double runtime_;
};

} /* namespace loco */
