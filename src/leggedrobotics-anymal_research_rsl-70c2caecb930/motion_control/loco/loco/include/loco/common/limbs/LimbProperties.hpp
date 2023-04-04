/*
 * LimbProperties.hpp
 *
 *  Created on: Dec 21, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"

// STL
#include <memory>

namespace loco {

class LimbProperties {
 public:
  LimbProperties();
  virtual ~LimbProperties() = default;

  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;

  virtual double getLimbMass() const;
  virtual void setLimbMass(double limbMass);

  virtual const Position& getPositionBaseToLimbComInBaseFrame() const;
  virtual void setPositionBaseToLimbComInBaseFrame(const Position& positionBaseToLimbComInBaseFrame);

  virtual double getMaximumLimbExtension() const = 0;
  virtual double getMinimumLimbExtension() const = 0;

 private:
  //! The total mass of the limb.
  double limbMass_;

  //! The center of the limb.
  Position positionBaseToLimbComInBaseFrame_;
};

using LimbPropertiesPtr = std::unique_ptr<LimbProperties>;

} /* namespace loco */
