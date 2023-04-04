/*
 * WholeBodyState.tpp
 *
 *  Created on: July 15, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// wholebody_romo
#include <whole_body_control_romo/WholeBodyStateRomo.hpp>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
WholeBodyStateRomo<ConcreteDescription_, RobotState_>::WholeBodyStateRomo(const RobotModel& model, const loco::WholeBody& wholeBody,
                                                                          const SupportJacobian& supportJacobian,
                                                                          const loco::TerrainModelBase& terrain,
                                                                          const ContactFlags& contactFlags)
    : model_(model),
      wholeBody_(wholeBody),
      supportJacobian_(supportJacobian),
      terrain_(terrain),
      contactFlags_(contactFlags),
      basetaskState_(model, wholeBody) {}

template <typename ConcreteDescription_, typename RobotState_>
bool WholeBodyStateRomo<ConcreteDescription_, RobotState_>::advance(double dt) {
  basetaskState_.advance(dt);
  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyStateRomo<ConcreteDescription_, RobotState_>::addVariablesToLog(const std::string& ns) const {
  basetaskState_.addVariablesToLog(ns);
  return true;
}

}  // namespace whole_body_control_romo