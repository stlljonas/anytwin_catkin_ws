/*
 * LimitsAnymal.hpp
 *
 *  Created on: Nov 7, 2018
 *      Author: Yvain de Viragh
 */

#pragma once

// model
#include "anymal_model/AnymalState.hpp"

// description
#include "anymal_description/AnymalDescription.hpp"

// romo
#include "romo/LimitsInterface.hpp"

namespace anymal_model {

class LimitsAnymal : public romo::LimitsInterface<CAD, AnymalState> {
 public:
  struct ActuatorLimitEntry {
    double minPosition_ = 0.0;
    double maxPosition_ = 0.0;
    double minVelocity_ = 0.0;
    double maxVelocity_ = 0.0;
    double minEffort_ = 0.0;
    double maxEffort_ = 0.0;
    double minCommandEffort_ = 0.0;
    double maxCommandEffort_ = 0.0;
    double minGearVelocity_ = 0.0;
    double maxGearVelocity_ = 0.0;
    double minCurrent_ = 0.0;
    double maxCurrent_ = 0.0;
  };

  struct JointLimitEntry {
    double minPosition_ = 0.0;
    double maxPosition_ = 0.0;
    double minVelocity_ = 0.0;
    double maxVelocity_ = 0.0;
    double minEffort_ = 0.0;
    double maxEffort_ = 0.0;
    double minCommandEffort_ = 0.0;
    double maxCommandEffort_ = 0.0;
  };

  using JointEnum = AD::JointEnum;
  using ActuatorEnum = AD::ActuatorEnum;
  using JointLimitsArray = std_utils::EnumArray<JointEnum, JointLimitEntry>;
  using ActuatorLimitsArray = std_utils::EnumArray<ActuatorEnum, ActuatorLimitEntry>;

  LimitsAnymal() = default;
  ~LimitsAnymal() override = default;

  void init() override;
  bool update(const JointPositions& jointPositions, const JointVelocities& jointVelocities) override;
  void printLimits() const override;

  virtual const ActuatorLimitEntry& getActuatorLimitEntry(ActuatorEnum actuator) const;
  virtual void setActuatorLimitEntry(ActuatorEnum actuator, ActuatorLimitEntry entry);

  virtual const JointLimitEntry& getJointLimitEntry(JointEnum joint) const;
  virtual void setJointLimitEntry(JointEnum joint, JointLimitEntry entry);

  double getActuatorMinPosition(ActuatorEnum actuator) const override;
  double getActuatorMaxPosition(ActuatorEnum actuator) const override;
  double getActuatorMinVelocity(ActuatorEnum actuator) const override;
  double getActuatorMaxVelocity(ActuatorEnum actuator) const override;
  double getActuatorMinEffort(ActuatorEnum actuator) const override;
  double getActuatorMaxEffort(ActuatorEnum actuator) const override;
  double getActuatorMinCommandEffort(ActuatorEnum actuator) const override;
  double getActuatorMaxCommandEffort(ActuatorEnum actuator) const override;
  virtual double getActuatorMinGearVelocity(ActuatorEnum actuator) const;
  virtual double getActuatorMaxGearVelocity(ActuatorEnum actuator) const;
  virtual double getActuatorMinCurrent(ActuatorEnum actuator) const;
  virtual double getActuatorMaxCurrent(ActuatorEnum actuator) const;

  virtual void setActuatorMinPosition(ActuatorEnum actuator, double minPosition);
  virtual void setActuatorMaxPosition(ActuatorEnum actuator, double maxPosition);
  virtual void setActuatorMinVelocity(ActuatorEnum actuator, double minVelocity);
  virtual void setActuatorMaxVelocity(ActuatorEnum actuator, double maxVelocity);
  virtual void setActuatorMinEffort(ActuatorEnum actuator, double minEffort);
  virtual void setActuatorMaxEffort(ActuatorEnum actuator, double maxEffort);
  virtual void setActuatorMinCommandEffort(ActuatorEnum actuator, double minCommandEffort);
  virtual void setActuatorMaxCommandEffort(ActuatorEnum actuator, double maxCommandEffort);
  virtual void setActuatorMinGearVelocity(ActuatorEnum actuator, double minGearVelocity);
  virtual void setActuatorMaxGearVelocity(ActuatorEnum actuator, double maxGearVelocity);
  virtual void setActuatorMinCurrent(ActuatorEnum actuator, double minCurrent);
  virtual void setActuatorMaxCurrent(ActuatorEnum actuator, double maxCurrent);

  double getJointMinPosition(JointEnum joint) const override;
  double getJointMaxPosition(JointEnum joint) const override;
  double getJointMinVelocity(JointEnum joint) const override;
  double getJointMaxVelocity(JointEnum joint) const override;
  double getJointMinEffort(JointEnum joint) const override;
  double getJointMaxEffort(JointEnum joint) const override;
  double getJointMinCommandEffort(JointEnum joint) const override;
  double getJointMaxCommandEffort(JointEnum joint) const override;

  virtual void setJointMinPosition(JointEnum joint, double minPosition);
  virtual void setJointMaxPosition(JointEnum joint, double maxPosition);
  virtual void setJointMinVelocity(JointEnum joint, double minVelocity);
  virtual void setJointMaxVelocity(JointEnum joint, double maxVelocity);
  virtual void setJointMinEffort(JointEnum joint, double minEffort);
  virtual void setJointMaxEffort(JointEnum joint, double maxEffort);
  virtual void setJointMinCommandEffort(JointEnum joint, double minCommandEffort);
  virtual void setJointMaxCommandEffort(JointEnum joint, double maxCommandEffort);

 protected:
  JointLimitsArray jointLimits_;
  ActuatorLimitsArray actuatorLimits_;
};

}  // namespace anymal_model
