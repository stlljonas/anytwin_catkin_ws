/*
 * GaitPatternBase.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include "locomotion_planner/gait_pattern/GaitPatternBase.hpp"

namespace locomotion_planner {

GaitPatternBase::GaitPatternBase(const Type& type) :
    type_(type),
    direction_(Direction::UNDEFINED)
{

}

GaitPatternBase::~GaitPatternBase()
{
}

const GaitPatternBase::Type & GaitPatternBase::getType() const
{
  return type_;
}

void GaitPatternBase::setDirection(const Direction& direction)
{
  direction_ = direction;
}

const GaitPatternBase::Direction& GaitPatternBase::getDirection() const
{
  return direction_;
}

std::ostream& operator<< (std::ostream& out, const GaitPatternBase::Direction& direction)
{
  switch (direction) {
    case GaitPatternBase::Direction::FORWARD:
      out << "Forward";
      return out;
    case GaitPatternBase::Direction::BACKWARD:
      out << "Backward";
      return out;
    case GaitPatternBase::Direction::LEFT:
      out << "Left";
      return out;
    case GaitPatternBase::Direction::RIGHT:
      out << "Right";
      return out;
    case GaitPatternBase::Direction::ROTATION_LEFT:
      out << "Rotation Left";
      return out;
    case GaitPatternBase::Direction::ROTATION_RIGHT:
      out << "Rotation Right";
      return out;
    case GaitPatternBase::Direction::FREE:
      out << "Free";
      return out;
    case GaitPatternBase::Direction::UNDEFINED:
      out << "Undefined";
      return out;
    default:
      out << "Error";
      return out;
  }
}

} /* namespace */
