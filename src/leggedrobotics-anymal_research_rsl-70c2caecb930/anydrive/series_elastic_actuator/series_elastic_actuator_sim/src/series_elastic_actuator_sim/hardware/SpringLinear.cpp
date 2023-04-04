/*!
 * @file    SpringLinear.cpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator sim
#include "series_elastic_actuator_sim/hardware/SpringLinear.hpp"


namespace series_elastic_actuator_sim {
namespace hardware {


SpringLinear::SpringLinear(double stiffness, double damping):
    SpringBase(),
    stiffness_(stiffness),
    damping_(damping)
{

}

SpringLinear::~SpringLinear()
{

}

bool SpringLinear::initialize(const SeActuatorState& state, double dt)
{
  return true;
}

bool SpringLinear::advance(SeActuatorState& stateOut, const SeActuatorState& stateIn, double dt) {
  double jointTorque = (stateIn.getGearPosition() - stateIn.getJointPosition())*stiffness_
                     + (stateIn.getGearVelocity() - stateIn.getJointVelocity())*damping_;
  stateOut.setJointTorque(jointTorque);
  return true;
}

double SpringLinear::getDamping() const
{
  return damping_;
}

void SpringLinear::setDamping(double damping)
{
  damping_ = damping;
}

double SpringLinear::getStiffness() const
{
  return stiffness_;
}

void SpringLinear::setStiffness(double stiffness)
{
  stiffness_ = stiffness;
}


} // hardware
} // series_elastic_actuator_sim
