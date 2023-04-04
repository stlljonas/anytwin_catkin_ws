/*
 * EndEffectorBase.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/end_effectors/EndEffectorEnum.hpp"
#include "loco/common/end_effectors/EndEffectorProperties.hpp"
#include "loco/common/end_effectors/EndEffectorStateDesired.hpp"
#include "loco/common/end_effectors/EndEffectorStateMeasured.hpp"
#include "loco/common/end_effectors/TimePoint.hpp"

// stl
#include <memory>
#include <ostream>
#include <unordered_map>

namespace loco {

class EndEffectorBase {
 public:
  using TimeInstant = unsigned int;
  using EndEffectorFrame = unsigned int;
  using DesiredStates = std::unordered_map<TimeInstant, std::unordered_map<EndEffectorFrame, EndEffectorStateDesiredPtr> >;
  using MeasuredStates = std::unordered_map<TimeInstant, std::unordered_map<EndEffectorFrame, EndEffectorStateMeasuredPtr> >;

 public:
  EndEffectorBase() = delete;
  explicit EndEffectorBase(EndEffectorPropertiesPtr&& endEffectorProperties, bool contactIsAtOrigin = true);
  virtual ~EndEffectorBase() = default;

  //! Initialize the class with a known state.
  virtual bool initialize(double dt) = 0;

  //! Update the class.
  virtual bool advance(double dt) = 0;

  // Advances all contact points.
  virtual bool advanceContactPoints(double dt) = 0;

  void setJointStatesLimb(DesiredJointStates* const desiredJointStatesLimb, MeasuredJointStates* const measuredJointStatesLimb);

  const EndEffectorStateDesired& getStateDesired(TimeInstant atTime = TimePoint::Now,
                                                 EndEffectorFrame atFrame = EndEffectorContactEnum::Contact) const;
  const EndEffectorStateMeasured& getStateMeasured(TimeInstant atTime = TimePoint::Now,
                                                   EndEffectorFrame atFrame = EndEffectorContactEnum::Contact) const;
  const EndEffectorProperties& getProperties() const;

  EndEffectorStateDesired* getStateDesiredPtr(TimeInstant atTime = TimePoint::Now,
                                              EndEffectorFrame atFrame = EndEffectorContactEnum::Contact);
  EndEffectorStateMeasured* getStateMeasuredPtr(TimeInstant atTime = TimePoint::Now,
                                                EndEffectorFrame atFrame = EndEffectorContactEnum::Contact);
  EndEffectorProperties* getPropertiesPtr();

  // Operational and Joint space mappings through forward and inverse kinematics.
  // IMPORTANT: The forward kinematics return the position of the Endeffector Frame (Origin)
  virtual loco::Position getPositionWorldToEndEffectorInWorldFrame(const JointPositions& jointPositions) = 0;
  virtual loco::Position getPositionWorldToEndEffectorInBaseFrame(const JointPositions& jointPositions) = 0;
  virtual loco::Position getPositionBaseToEndEffectorInBaseFrame(const JointPositions& jointPositions) = 0;

  // IMPORTANT: The inverse kinematics take the position/velocity of the Endeffector Frame (Origin)
  virtual JointPositions getJointPositionsFromPositionBaseToEndEffectorInBaseFrameIteratively(
      const loco::Position& positionBaseToEndEffectorInBaseFrame) = 0;
  virtual JointPositions getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(
      const Position& positionBaseToEndEffectorInBaseFrame) = 0;
  virtual JointVelocities getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(
      const LinearVelocity& linearVelocityBaseToEndEffectorInBaseFrame) = 0;

  /*! Computes the 3xDOF Jacobian matrix, given a set of joint angles of the leg
   *
   * @param jointPositions The joint angles of the leg in consideration
   * @return The Jacobian of size 3xN, where N = size of generalized velocity vector 'u'
   */
  virtual Eigen::MatrixXd getTranslationJacobianBaseToEndEffectorInBaseFrameFromJointAngles(const JointPositions& jointPositions) = 0;

  virtual void addVariablesToLog(bool /* update */) {}
  friend std::ostream& operator<<(std::ostream& out, const EndEffectorBase& /* torso */) { return out; }

  virtual bool isInContact() const = 0;
  virtual bool isSlipping() const = 0;

 protected:
  DesiredStates endEffectorStateDesired_;
  MeasuredStates endEffectorStateMeasured_;
  EndEffectorPropertiesPtr endEffectorProperties_;
  const bool contactIsAtOrigin_;
};

using EndEffectorBasePtr = std::unique_ptr<EndEffectorBase>;

} /* namespace loco */
