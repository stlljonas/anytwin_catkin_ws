/*
 * IndicatorContainer.hpp
 *
 *  Created on: Aug 08, 2019
 *      Author: Fabian Jenelten
 */

// std.
#include <memory>
#include <vector>

// state_checker.
#include "state_checker/IndicatorBase.hpp"

// message logger.
#include "message_logger/message_logger.hpp"

#pragma once

class TiXmlHandle;
namespace loco {
namespace state_checker {

class IndicatorContainer {
 public:
  IndicatorContainer() : indicators_() {}

  ~IndicatorContainer() = default;

  bool initialize(double dt, WholeBody& wholeBody, TerrainModelBase& terrain) {
    for (auto const& indicator : indicators_) {
      if (!indicator->initialize(dt, wholeBody, terrain)) {
        MELO_WARN_STREAM("[IndicatorContainer::initialize] Failed initialize indicator " << indicator->getName() << ".");
        return false;
      }
    }
    return true;
  }

  bool loadParameters(const TiXmlHandle& handle) {
    for (auto const& indicator : indicators_) {
      if (!indicator->loadParameters(handle)) {
        MELO_WARN_STREAM("[IndicatorContainer::loadParameters] Failed to load parameters for indicator " << indicator->getName() << ".");
        return false;
      }
    }
    return true;
  }

  double computeIndicatorValue(double dt, WholeBody& wholeBody, TerrainModelBase& terrain) {
    double indicatorValue = 0.0;
    for (auto const& indicator : indicators_) {
      indicatorValue += indicator->computeIndicatorValue(dt, wholeBody, terrain);
    }
    return std::fmin(indicatorValue, 1.0);
  }

  void clear() { indicators_.clear(); }

  void addIndicator(IndicatorBase* indicator) { indicators_.emplace_back(std::unique_ptr<IndicatorBase>(indicator)); }

  IndicatorBase* getIndicator(const std::string& name) {
    for (auto const& indicator : indicators_) {
      if (indicator->getName() == name) {
        return indicator.get();
      }
    }

    MELO_WARN_STREAM("[IndicatorContainer::loadParameters] Could not find indicator with name " << name << ".");
    return nullptr;
  }

 private:
  //! Vector indicator functions.
  std::vector<std::unique_ptr<IndicatorBase>> indicators_;
};

} /* namespace state_checker */
} /* namespace loco */
