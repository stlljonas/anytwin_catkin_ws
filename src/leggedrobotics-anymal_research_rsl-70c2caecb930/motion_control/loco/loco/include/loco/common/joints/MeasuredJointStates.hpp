/*
 * MeasuredJointStates.hpp
 *
 *  Created on: Feb 13, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

#include "loco/common/joints/JointStates.hpp"

namespace loco {

class MeasuredJointStates : public JointStates {
 public:
  explicit MeasuredJointStates(const unsigned int nJoints);
  ~MeasuredJointStates() override = default;

  virtual const JointTorques& getGravityJointTorques() const;
  virtual void setGravityJointTorques(const JointTorques& gravityJointTorques);
  virtual void setGravityJointTorque(const unsigned int index, const JointTorque& gravityJointTorque);

  //! Returns the vector of minimum joint position values permitted.
  virtual const JointPositions& getJointMinPositions() const;

  /*! Sets the minimum joint position value from the model.
   *
   * @param index The index of the joint in the leg.
   * @param jointMinPosition The lower limit of the permissible joint position value.
   */
  virtual void setJointMinPosition(const unsigned int index, const JointPosition& jointMinPosition);

  //! Returns the vector of maximum joint position values permitted.
  virtual const JointPositions& getJointMaxPositions() const;

  /*! Sets the maximum joint position value from the model.
   *
   * @param index The index of the joint in the leg.
   * @param jointMaxPosition The upper limit of the permissible joint position value.
   */
  virtual void setJointMaxPosition(const unsigned int index, const JointPosition& jointMaxPosition);

  //! Returns the vector of minimum joint velocity values permitted.
  virtual const JointVelocities& getJointMinVelocities() const;

  /*! Sets the minimum joint velocity value from the model.
   *
   * @param index The index of the joint in the leg.
   * @param jointMinVelocity The lower limit of the permissible joint velocity value.
   */
  virtual void setJointMinVelocity(const unsigned int index, const JointVelocity& jointMinVelocity);

  //! Returns the vector of maximum joint velocity values permitted.
  virtual const JointVelocities& getJointMaxVelocities() const;

  /*! Sets the maximum joint velocity value from the model.
   *
   * @param index The index of the joint in the leg.
   * @param jointMaxVelocity The upper limit of the permissible joint velocity value.
   */
  virtual void setJointMaxVelocity(const unsigned int index, const JointVelocity& jointMaxVelocity);

 protected:
  //! The vector of joint torques due to gravity.
  JointTorques gravityJointTorques_;

  //! The vector of minimum permissible joint positions.
  JointPositions jointMinPositions_;

  //! The vector of maximum permissible joint positions.
  JointPositions jointMaxPositions_;

  //! The vector of minimum permissible joint velocities.
  JointVelocities jointMinVelocities_;

  //! The vector of maximum permissible joint velocities.
  JointVelocities jointMaxVelocities_;
};

} /* namespace loco */
