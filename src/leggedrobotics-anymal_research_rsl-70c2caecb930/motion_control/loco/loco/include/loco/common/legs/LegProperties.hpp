/*
 * LegProperties.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// loco
#include "loco/common/limbs/LimbProperties.hpp"
#include "loco/common/typedefs.hpp"

// STL
#include <memory>

namespace loco {

class LegProperties : public LimbProperties {
 public:
  LegProperties() = default;
  ~LegProperties() override = default;

  virtual const Position& getDesiredDefaultSteppingPositionHipToFootInControlFrame() const;
  virtual void setDesiredDefaultSteppingPositionHipToFootInControlFrame(const Position& position);

 private:
  //! default stepping offset with respect to the hip
  Position desiredDefaultSteppingPositionHipToFootInControlFrame_;
};

using LegPropertiesPtr = std::unique_ptr<LegProperties>;

} /* namespace loco */
