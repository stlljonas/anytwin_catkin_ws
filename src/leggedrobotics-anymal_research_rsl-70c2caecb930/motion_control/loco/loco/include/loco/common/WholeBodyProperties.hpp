/*
 * WholeBodyProperties.hpp
 *
 *  Created on: Jan 25, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

#include <memory>

namespace loco {

class WholeBodyProperties {
 public:
  WholeBodyProperties();
  virtual ~WholeBodyProperties() = default;

  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;

  void setTotalMass(double totalMass);
  double getTotalMass() const;

 private:
  double totalMass_;
};

using WholeBodyPropertiesPtr = std::unique_ptr<WholeBodyProperties>;

} /* namespace loco */
