/*
 * LimbBase.hpp
 *
 *  Created on: Dec 13, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/end_effectors/EndEffectorBase.hpp"
#include "loco/common/limbs/LimbLink.hpp"
#include "loco/common/limbs/LimbLinkGroup.hpp"
#include "loco/common/limbs/LimbProperties.hpp"
#include "loco/common/limbs/LimbStateDesired.hpp"
#include "loco/common/limbs/LimbStateMeasured.hpp"
#include "loco/common/limbs/LimbStrategy.hpp"
#include "loco/common/typedefs.hpp"

namespace loco {

class LimbBase : public ModuleBase {
 public:
  explicit LimbBase(const std::string name, const unsigned int numDofLimb, LimbPropertiesPtr&& properties,
                    EndEffectorBasePtr&& endEffector);

  explicit LimbBase(const std::string name, const unsigned int numDofLimb, LimbPropertiesPtr&& properties, EndEffectorBasePtr&& endEffector,
                    LimbStateMeasuredPtr&& stateMeasured, LimbStateDesiredPtr&& stateDesired);

  ~LimbBase() override = default;

  //! Dimension of the leg joint space, i.e. number of degrees of freedom of the limb.
  virtual unsigned int getNumDofLimb() const;

  //! Dimension of the velocity joint space of the whole robot.
  virtual unsigned int getNumDofU() const = 0;

  //! Get the branch identifier of this limb.
  virtual unsigned int getBranchUInt() const = 0;

  //! Get the limb identifier of this limb.
  virtual unsigned int getLimbUInt() const = 0;

  //! Get the identifier of this limb.
  virtual unsigned int getId() const = 0;

  /*!Add variables to log.
   * @return true if successful.
   */
  bool addVariablesToLog(const std::string& logNameSpace) const override;

  //! Get a vector of joint positions initialized to zero.
  JointPositions getInitializedJointPositions() const;

  //! Get a vector of joint velocities initialized to zero.
  JointVelocities getInitializedJointVelocities() const;

  //! Get a vector of joint accelerations initialized to zero.
  JointAccelerations getInitializedJointAccelerations() const;

  //! Get a vector of joint torques initialized to zero.
  JointTorques getInitializedJointTorques() const;

  //! Get a vector of joint control modes initialized to freeze control mode.
  JointControlModes getInitializedJointControlModes() const;

  /*! Get a Jacobian matrix initialized to zero. The dimension of the column space of this Jacobian
   * is the same as the number of DoFs of the entire robot.
   */
  TranslationJacobian getInitializedTranslationJacobian() const;

  /*! Get a Jacobian matrix initialized to zero. The dimension of the column space of this Jacobian
   * is the same as the number of DoFs of a single limb.
   */
  TranslationJacobianLimb getInitializedTranslationJacobianLimb() const;

  /*! Get a Jacobian matrix initialized to zero. The dimension of the column space of this Jacobian
   * is the same as the number of DoFs of the entire robot.
   */
  RotationJacobian getInitializedRotationJacobian() const;

  /*! Get a Jacobian matrix initialized to zero. The dimension of the column space of this Jacobian
   * is the same as the number of DoFs of a single limb.
   */
  RotationJacobianLimb getInitializedRotationJacobianLimb() const;

  //! Get a vector of joint names initialized to empty strings.
  JointNames getInitializedJointNames() const;

  //! Resize and initialize a vector of joint positions to zero.
  void populateJointPositions(JointPositions& joints) const;

  //! Resize and initialize a vector of joint velocities to zero.
  void populateJointVelocities(JointVelocities& jointVelocities) const;

  //! Resize and initialize a vector of joint accelerations to zero.
  void populateJointAccelerations(JointAccelerations& jointAccelerations) const;

  //! Resize and initialize a vector of joint torques to zero.
  void populateJointTorques(JointTorques& jointTorques) const;

  //! Resize and initialize a vector of joint control modes to freeze control mode.
  void populateJointControlModes(JointControlModes& jointControlModes) const;

  /*! Resize and initialize a Jacobian matrix to zero. The dimension of the column space of this Jacobian
   * is the same as the number of DoFs of a single limb.
   */
  void populateTranslationJacobianLimb(TranslationJacobianLimb& translationJacobianLimb) const;

  /*! Resize and initialize a Jacobian matrix to zero. The dimension of the column space of this Jacobian
   * is the same as the number of DoFs of the entire robot.
   */
  void populateTranslationJacobian(TranslationJacobian& translationJacobian) const;

  /*! Resize and initialize a Jacobian matrix to zero. The dimension of the column space of this Jacobian
   * is the same as the number of DoFs of a single limb.
   */
  void populateRotationJacobianLimb(RotationJacobianLimb& rotationJacobianLimb) const;

  /*! Resize and initialize a Jacobian matrix to zero. The dimension of the column space of this Jacobian
   * is the same as the number of DoFs of the entire robot.
   */
  void populateRotationJacobian(RotationJacobian& rotationJacobian) const;

  /*! Resize and initialize a Jacobian matrix to zero. The dimension of the column space of this Jacobian
   * is the same as the number of DoFs of the entire robot.
   */
  void populateSpatialJacobian(SpatialJacobian& spatialJacobian) const;

  //! Resize and initialize a vector of joint names to empty strings.
  void populateJointNames(JointNames& jointNames) const;

  LimbStateDesired* getLimbStateDesiredPtr();
  const LimbStateDesired& getLimbStateDesired() const;

  LimbStateMeasured* getLimbStateMeasuredPtr();
  const LimbStateMeasured& getLimbStateMeasured() const;

  EndEffectorBase* getEndEffectorPtr();
  const EndEffectorBase& getEndEffector() const;

  LimbProperties* getLimbPropertiesPtr();
  const LimbProperties& getLimbProperties() const;

  LimbStrategy* getLimbStrategyPtr();
  const LimbStrategy& getLimbStrategy() const;

  //!@returns list of links.
  LimbLinkGroup* getLinksPtr();
  const LimbLinkGroup& getLinks() const;

  //!@returns vecot of joint names.
  JointNames* getJointNamesPtr();
  const JointNames& getJointNames() const;

  /*! @returns the load factor between 0 and 1, which indicates how much the limb can/should be loaded.
   *  If it is one, the limb can/should be fully loaded.
   *  If it is zero, the limb cannot/shouldn't be loaded at all.
   */
  double getLoadFactor() const;

  /*!
   * Change how much a limb should be loaded.
   * @param loadFactor sets the factor how much the limb should be loaded
   *        (related to the unconstrained case without user specified load
   *        factors), value in the interval [0, 1] where 0: unloaded
   *        and 1: completely loaded.
   */
  void setLoadFactor(double loadFactor);

  void setFrictionModulation(double frictionModulation);
  double getFrictionModulation() const;

 protected:
  //! Dof
  unsigned int numDofLimb_;

  //! Limb properties (e.g. mass, CoM)
  LimbPropertiesPtr limbProperties_;

  //! Endeffector of the limb (e.g foot, wheel)
  EndEffectorBasePtr endEffector_;

  //! Desired state of the limb.
  LimbStateDesiredPtr limbStateDesired_;

  //! Measured state of the limb.
  LimbStateMeasuredPtr limbStateMeasured_;

  //! A manager of control strategies for this limb.
  LimbStrategyPtr limbStrategy_;

  /** This is a list of links.
   *  IMPORTANT: The links are owned by this class (LimbBase). It also deletes this links at the end of its lifetime.
   */
  LimbLinkGroup links_;

  //! The names of the limb joints.
  JointNames jointNames_;

  /*! This limb should be loaded only by this factor, which is between 0 and 1.
   *  If it is 1, the limb can/should be fully loaded.
   *  If it is 0, the limb cannot/shouldn't be loaded at all.
   */
  double loadFactor_;

  //! Modulate the friction cone for this limb.
  double frictionModulation_;
};

} /* namespace loco */
