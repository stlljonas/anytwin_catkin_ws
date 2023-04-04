/*
 * StateCheckerBase.hpp
 *
 *  Created on: Aug 08, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

// loco.
#include "loco/common/ModuleBase.hpp"

// signal logger.
#include "signal_logger/signal_logger.hpp"

class TiXmlHandle;

namespace loco {
namespace state_checker {

class StateCheckerBase: public ModuleBase {
  public:
    explicit StateCheckerBase() :
      indicatorValue_(0.0),
      isSafe_(true) { }

    ~StateCheckerBase() override = default;

    bool initialize(double dt) override {
      indicatorValue_ = 0.0;
      isSafe_ = true;
      return true;
    }

    bool advance(double dt) override {
      return true;
    }

    bool addVariablesToLog(const std::string & ns = "") const override {
      signal_logger::add(indicatorValue_, "/state_checker/indication_value", "/loco", "[-]");
      signal_logger::add(isSafe_, "/state_checker/is_safe", "/loco", "[bool]");
      return true;
    }

    double getConditionNumber() const noexcept { return indicatorValue_; }
    bool getIsSafe() const noexcept { return isSafe_; }

  protected:

    //! Stability indication (0: ok, 1: unsafe).
    double indicatorValue_;

    //! Hysteresis thresholded indicator value (true: ok, false: unsafe).
    bool isSafe_;
};

} /* namespace state_checker */
} /* namespace loco */
