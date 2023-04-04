/*
 * IndicatorBase.hpp
 *
 *  Created on: Aug 08, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

// loco.
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"

class TiXmlHandle;

namespace loco {
namespace state_checker {

class IndicatorBase {
 public:
  IndicatorBase() : name_("") {}
  virtual ~IndicatorBase() = default;

  virtual bool initialize(double /*dt*/, WholeBody& /*wholeBody*/, TerrainModelBase& /*terrain*/) { return true; }
  virtual bool loadParameters(const TiXmlHandle& /*handle*/) { return true; };
  virtual double computeIndicatorValue(double dt, WholeBody& wholeBody, TerrainModelBase& terrain) = 0;

  const std::string getName() const noexcept { return name_; }

 protected:
  //! Name of the indicator.
  std::string name_;
};

} /* namespace state_checker */
} /* namespace loco */
