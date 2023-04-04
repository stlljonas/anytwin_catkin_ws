/*!
 * @file	 HandRomo.tpp
 * @author Markus Staeuble
 * @date	 Jun, 2018
 */

#pragma once

// romo_measurements
#include "romo_measurements/end_effectors/HandRomo.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
HandRomo<ConcreteDescription_, RobotState_, EndeffectorBase_ >::HandRomo(
    BodyEnum bodyEnum, unsigned int numFingers, const std::string& name,
    const RobotModel& model, loco::EndEffectorPropertiesPtr&& endeffectorProperties,
    const ContactsMap & contactPointsMap,
    const std::vector<TimeInstant>& timeInstants,
    bool autoAdvanceOfContactPoints)
    : EndEffectorRomo(bodyEnum, numFingers, name, model, std::move(endeffectorProperties), contactPointsMap, timeInstants, autoAdvanceOfContactPoints)
{ }

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
HandRomo<ConcreteDescription_, RobotState_, EndeffectorBase_ >::HandRomo(
    BodyEnum bodyEnum, unsigned int numFingers, const std::string& name, const RobotModel& model,
    const ContactsMap & contactPointsMap,
    const std::vector<TimeInstant>& timeInstants,
    bool autoAdvanceOfContactPoints)
    : EndEffectorRomo(bodyEnum, numFingers, name, model, contactPointsMap, timeInstants, autoAdvanceOfContactPoints)
{ }

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
bool HandRomo<ConcreteDescription_, RobotState_, EndeffectorBase_ >::initialize(double dt) {
  if(!EndEffectorRomo::initialize(dt)) {
    MELO_WARN_STREAM("[HandRomo]: Could not initialize EndEffectorRomo!");
    return false;
  }

  return true;
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
bool HandRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::advance(double dt) {
  bool success = true;
  success &= EndEffectorRomo::advance(dt);
  success &= advanceFingers();
  return success;
}

}  // namespace romo_measurements
