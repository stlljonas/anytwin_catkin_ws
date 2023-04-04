/*!
 * @file	 FootRomo.tpp
 * @author Dario Bellicoso
 * @date	 Jan, 2018
 */

#pragma once

// romo_measurements
#include "romo_measurements/end_effectors/FootRomo.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
FootRomo<ConcreteDescription_, RobotState_, EndeffectorBase_ >::FootRomo(
    BodyEnum bodyEnum, const std::string& name,
    const RobotModel& model, loco::EndEffectorPropertiesPtr&& endeffectorProperties,
    const ContactsMap & contactPointsMap,
    const std::vector<TimeInstant>& timeInstants,
    bool autoAdvanceOfContactPoints)
    : EndEffectorRomo(bodyEnum, name, model, std::move(endeffectorProperties), contactPointsMap, timeInstants, autoAdvanceOfContactPoints)
{ }

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
FootRomo<ConcreteDescription_, RobotState_, EndeffectorBase_ >::FootRomo(
    BodyEnum bodyEnum, const std::string& name, const RobotModel& model,
    const ContactsMap & contactPointsMap,
    const std::vector<TimeInstant>& timeInstants,
    bool autoAdvanceOfContactPoints)
    : EndEffectorRomo(bodyEnum, name, model, contactPointsMap, timeInstants, autoAdvanceOfContactPoints)
{ }

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
bool FootRomo<ConcreteDescription_, RobotState_, EndeffectorBase_ >::initialize(double dt) {
  if(!EndEffectorRomo::initialize(dt)) {
    MELO_WARN_STREAM("[FootRomo]: Could not initialize EndEffectorRomo!");
    return false;
  }

  for(auto & endEffectorStateDesired : this->endEffectorStateDesired_[loco::TimePoint::Now]) {
    auto& footStateDesired = static_cast<loco::FootBaseStateDesired&>(*endEffectorStateDesired.second);
    footStateDesired.setPositionWorldToFootholdInWorldFrame(endEffectorStateDesired.second->getPositionWorldToEndEffectorInWorldFrame());
  }

  return true;
}

}  // namespace romo_measurements
