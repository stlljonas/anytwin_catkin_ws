/*
 * EndEffectorProperties.hpp
 *
 *  Created on: Dec 21, 2016
 *      Author: Gabriel Hottiger, Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/typedefs.hpp>

// stl
#include <memory>

namespace loco {

class EndEffectorProperties {
 public:
  EndEffectorProperties();
  explicit EndEffectorProperties(unsigned int numberOfContactConstraints);
  virtual ~EndEffectorProperties() = default;

  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;

  virtual double getMass() const = delete;
  virtual void setMass(double mass) = delete;

  virtual const Position& getBaseToCenterOfMassPositionInBaseFrame() const = delete;
  virtual void setBaseToCenterOfMassPositionInBaseFrame(const Position& centerOfMassInBaseFrame) = delete;

  //! Get the number of contact constraints that can be imposed by this end-effector.
  // E.g.: point foot --> 3 constraints
  //       hand       --> 6 constraints
  virtual unsigned int getNumberOfContactConstraints() const;

 private:
  //! The mass of the end-effector.
  double mass_;

  //! The center of the total mass of the end-effector.
  Position positionBaseToCenterOfMassInBaseFrame_;

  //! The number of contact constraints that this end-effector. The default is 3.
  unsigned int numberOfContactConstraints_;
};

using EndEffectorPropertiesPtr = std::unique_ptr<EndEffectorProperties>;
}  // namespace loco
