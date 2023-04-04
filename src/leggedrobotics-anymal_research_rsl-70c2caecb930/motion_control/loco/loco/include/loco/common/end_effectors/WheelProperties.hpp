/*
 * WheelProperties.hpp
 *
 *  Created on: Dec 21, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

#include "loco/common/end_effectors/EndEffectorProperties.hpp"

namespace loco {

class WheelProperties : public EndEffectorProperties {
 public:
  WheelProperties();
  ~WheelProperties() override = default;

  virtual double getDiameter() const;
  virtual void setDiameter(double diameter);

 private:
  //! The diameter of the wheel
  double diameter_;
};

using WheelPropertiesPtr = std::unique_ptr<WheelProperties>;

} /* namespace loco */
