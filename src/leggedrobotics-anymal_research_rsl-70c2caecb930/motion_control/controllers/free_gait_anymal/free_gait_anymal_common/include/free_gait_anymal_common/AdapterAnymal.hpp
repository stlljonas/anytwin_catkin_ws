/*
 * AdapterAnymal.hpp
 *
 *  Created on: Sep 1, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/executor/AdapterBase.hpp>

// Anymal model
#include <anymal_model/AnymalModel.hpp>

// Measurements
#include <any_measurements/PoseWithCovariance.hpp>
#include <any_measurements/Time.hpp>

// Geometry utils
#include <geometry_utils/TransformListener.hpp>

// STD
#include <memory>

namespace free_gait {

//! Not thread-safe for Anymal Model!
class AdapterAnymal : public AdapterBase
{
 public:
  using AD = anymal_description::AnymalDescription;

  AdapterAnymal();
  //! Constructs but does not fully initalize the adapter
  explicit AdapterAnymal(anymal_model::AnymalModel& anymalModel);
  //! Constructs and initializes the adapter
  explicit AdapterAnymal(anymal_model::AnymalModel& anymalModel, std::unique_ptr<geometry_utils::TransformListener>& tfListener);

  ~AdapterAnymal() override = default;

  //! Initialize (if not done through constructor).
  void initialize(anymal_model::AnymalModel& anymalModel);
  void initialize(anymal_model::AnymalModel& anymalModel, std::unique_ptr<geometry_utils::TransformListener>& tfListener);
  void initialize(anymal_model::AnymalModel* anymalModel, std::unique_ptr<geometry_utils::TransformListener>& tfListener);

  anymal_model::AnymalModel& getAnymalModel();
  geometry_utils::TransformListener& getTfListener();
  bool setTfListener(std::unique_ptr<geometry_utils::TransformListener>& tfListener);

  //! Copying data from real robot to free gait state.
  virtual bool resetExtrasWithRobot(const StepQueue& stepQueue, State& state);
  virtual bool updateExtrasBefore(const StepQueue& stepQueue, State& state);
  virtual bool updateExtrasAfter(const StepQueue& stepQueue, State& state);

  //! Selecting inverse kinematic methods
  void setIterativeInverseKinematics();
  void setAnalyticInverseKinematics();

  //! State independent functions.
  const std::vector<LimbEnum>& getLimbs() const;
  const std::vector<BranchEnum>& getBranches() const;
  LimbEnum getLimbEnumFromLimbString(const std::string& limb) const;
  std::string getLimbStringFromLimbEnum(const LimbEnum& limb) const;
  std::string getBaseString() const;
  JointNodeEnum getJointNodeEnumFromJointNodeString(const std::string& jointNode) const;
  std::string getJointNodeStringFromJointNodeEnum(const JointNodeEnum& jointNode) const;
  bool getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      const Position& positionBaseToFootInBaseFrame, const LimbEnum& limb,
      JointPositionsLeg& jointPositions) const;
  Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb,
                                            const JointPositionsLeg& jointPositions) const;
  Position getPositionBaseToHipInBaseFrame(const LimbEnum& limb) const;

  //! Reading state of the robot.
  virtual bool isExecutionOk() const;
  virtual bool isLegGrounded(const LimbEnum& limb) const;
  JointPositionsLeg getJointPositionsForLimb(const LimbEnum& limb) const;
  JointPositions getAllJointPositions() const;
  JointVelocitiesLeg getJointVelocitiesForLimb(const LimbEnum& limb) const;
  JointVelocities getAllJointVelocities() const;
  JointAccelerationsLeg getJointAccelerationsForLimb(const LimbEnum& limb) const;
  JointAccelerations getAllJointAccelerations() const;
  JointEffortsLeg getJointEffortsForLimb(const LimbEnum& limb) const;
  JointEfforts getAllJointEfforts() const;
  Position getPositionWorldToBaseInWorldFrame() const;
  RotationQuaternion getOrientationBaseToWorld() const;
  LinearVelocity getLinearVelocityBaseInWorldFrame() const;
  LocalAngularVelocity getAngularVelocityBaseInBaseFrame() const;
  LinearAcceleration getLinearAccelerationBaseInWorldFrame() const;
  AngularAcceleration getAngularAccelerationBaseInBaseFrame() const;
  Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb) const;
  Position getPositionWorldToFootInWorldFrame(const LimbEnum& limb) const;
  Position getCenterOfMassInWorldFrame() const;

  //! Reading control setups
  virtual ControlSetup getControlSetup(const BranchEnum& branch) const;
  virtual ControlSetup getControlSetup(const LimbEnum& limb) const;

  //! Dealing with reference frames
  const std::string& getWorldFrameId() const;
  const std::string& getBaseFrameId() const;
  bool frameIdExists(const std::string& frameId) const;
  void getAvailableFrameTransforms(std::vector<std::string>& frameTransforms) const;
  bool getFrameTransform(const std::string& frameId, Pose& pose) const;

  //! State depending on real robot.
  JointVelocitiesLeg getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame(
      const LimbEnum& limb, const LinearVelocity& endEffectorLinearVelocityInWorldFrame) const;
  JointAccelerationsLeg getJointAccelerationsFromEndEffectorLinearAccelerationInWorldFrame(
      const LimbEnum& limb, const LinearAcceleration& endEffectorLinearAccelerationInWorldFrame) const;
  LinearVelocity getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb,
                                                                 const JointVelocitiesLeg& jointVelocities,
                                                                 const std::string& frameId) const;

  //! Hook to write data to internal robot representation from state.
  //! Sets the anymal model from the state, instead of other way around.
  bool setInternalDataFromState(const State& state, bool updateContacts = true, bool updatePosition = true,
                                bool updateVelocity = true, bool updateAcceleration = false) const;
  void createCopyOfState() const;
  void resetToCopyOfState() const;

 protected:
  virtual bool updateFrameTransforms();

  anymal_model::AnymalModel* anymalModel_ = nullptr;
  std::vector<LimbEnum> limbs_;
  std::vector<BranchEnum> branches_;
  std::unique_ptr<anymal_model::AnymalState> copyOfState_;
  std::unique_ptr<geometry_utils::TransformListener> tfListener_; //TODO(paco): ev. move to general Free Gait
  bool useAnalyticInverseKinematics_ = false;

 private:
  const std::string worldFrameId_;
  const std::string baseFrameId_;
};

} /* namespace free_gait */
