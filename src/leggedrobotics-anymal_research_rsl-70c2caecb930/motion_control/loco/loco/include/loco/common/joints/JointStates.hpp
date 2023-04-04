/*
 * JointState.hpp
 *
 *  Created on: Feb 13, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

#include "loco/common/typedefs.hpp"

namespace loco {

class JointStates {
 public:
  explicit JointStates(const unsigned int nJoints);
  virtual ~JointStates() = default;

  virtual const JointControlModes& getJointControlModes() const;
  virtual void setJointControlModes(const JointControlModes& jointControlModes);
  virtual void setJointControlMode(const unsigned int index, const JointControlMode& jointControlMode);

  virtual const JointPositions& getJointPositions() const;
  virtual void setJointPositions(const JointPositions& jointPositions);
  virtual void setJointPosition(const unsigned int index, const JointPosition& jointPosition);

  virtual const JointVelocities& getJointVelocities() const;
  virtual void setJointVelocities(const JointVelocities& jointVelocities);
  virtual void setJointVelocity(const unsigned int index, const JointVelocity& jointVelocity);

  virtual const JointAccelerations& getJointAccelerations() const;
  virtual void setJointAccelerations(const JointAccelerations& jointAccelerations);
  virtual void setJointAcceleration(const unsigned int index, const JointAcceleration& jointAcceleration);

  virtual const JointTorques& getJointTorques() const;
  virtual void setJointTorques(const JointTorques& jointTorques);
  virtual void setJointTorque(const unsigned int index, const JointTorque& jointTorque);

 protected:
  //! The number of joints represented by this container
  const unsigned int nJoints_;
  //! The vector of joint positions.
  JointPositions jointPositions_;
  //! The vector of joint velocities.
  JointVelocities jointVelocities_;
  //! The vector of joint accelerations.
  JointAccelerations jointAccelerations_;
  //! The vector of joint torques.
  JointTorques jointTorques_;
  //! The vector of joint control modes.
  JointControlModes jointControlModes_;
};

} /* namespace loco */
