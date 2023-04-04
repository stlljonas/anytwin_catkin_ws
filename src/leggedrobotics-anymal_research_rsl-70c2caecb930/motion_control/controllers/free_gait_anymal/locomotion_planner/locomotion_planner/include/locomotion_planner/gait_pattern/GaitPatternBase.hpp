/*
 * GaitPattern.hpp
 *
 *  Created on: Mar 10, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "locomotion_planner/common/type_defs.hpp"

#include <vector>
#include <map>

namespace locomotion_planner {

class GaitPatternBase
{
 public:
  enum class Type {
    CRAWLING,
    LONGITUDINAL_CRAWLING,
    FREE
  };

  enum class Direction {
    UNDEFINED,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    ROTATION_LEFT,
    ROTATION_RIGHT,
    FREE
  };

  GaitPatternBase(const Type& type);
  virtual ~GaitPatternBase();

  const Type& getType() const;

  void setDirection(const Direction& direction);
  const Direction& getDirection() const;

  virtual void setFirstStep(const LimbEnum& limb) = 0;
  virtual void advance() = 0;
  virtual const LimbEnum getCurrentLimb() const = 0;
  virtual bool startedNewCycle() const = 0;
  virtual bool endOfCycle() const = 0;

  virtual const std::vector<LimbEnum> getGaitPattern(const Direction& direction) = 0;

 private:
  //! Gait pattern type.
  Type type_;

  //! Motion direction.
  Direction direction_;
};

std::ostream& operator<< (std::ostream& out, const GaitPatternBase::Direction& direction);

} /* namespace locomotion_planner */
