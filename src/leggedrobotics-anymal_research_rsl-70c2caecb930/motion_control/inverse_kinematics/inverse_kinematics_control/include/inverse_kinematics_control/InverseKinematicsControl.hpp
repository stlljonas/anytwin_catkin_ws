/**
 * @file        InverseKinematicsControl.hpp
 * @authors     Fabian Jenelten
 * @date        Mar 23, 2020
 * @affiliation ETH RSL
 * @brief       This package provides a module that computes joint positions and joint velocities through inverse kinematics
 *              and inverse differential kinematics. The module can be used as impedance controller or as a light-weight,
 *              stand-alone tracking controller. In the latter case, joint torques are set to zero and the joint control modes are
 * overwritten.
 */

#pragma once

// loco.
#include <loco/common/typedefs.hpp>
#include <loco/motion_control/MotionControllerBase.hpp>

// std utils.
#include <std_utils/ConsecutiveEnum.hpp>

// anymal description.
#include <anymal_description/AnymalDescription.hpp>

// basic filters.
#include <basic_filters/FirstOrderFilter.hpp>

// std.
#include <memory>

class TiXmlHandle;

namespace inverse_kinematics_control {

/*!
 * Inverse kinematics computes joint targets from foot targets (positions and velocities).
 *
 * Inputs:
 * - for each ``limb->endEffector->desiredState``:
 *   - getPositionWorldToEndEffectorInWorldFrame()
 *   - getLinearVelocityEndEffectorInWorldFrame()
 * - when ``useDesiredTorsoState`` is false, in ``torso->measuredState``:
 *   - getOrientationWorldToBase()
 *   - getPositionWorldToBaseInWorldFrame()
 *   - getLinearVelocityBaseInBaseFrame()
 *   - getAngularVelocityBaseInBaseFrame()
 * - when ``useDesiredTorsoState`` is true, in ``torso->desiredState``:
 *   - getOrientationControlToBase()
 *   - getPositionWorldToBaseInWorldFrame()
 *   - getLinearVelocityTargetInControlFrame()
 *   - getAngularVelocityBaseInControlFrame()
 * - when ``useDesiredTorsoState`` is true, ``torso->measuredState->getOrientationWorldToControl()``.
 *
 * Outputs:
 * - for each ``limb->desiredState``:
 *   - setJointPositions()
 *   - setJointVelocities()
 *
 * Advanced by: main controller module.
 *
 */
class InverseKinematicsControl : public loco::MotionControllerBase {
 public:
  using Base = MotionControllerBase;
  using AD = anymal_description::AnymalDescription;

  //! Default Constructor.
  explicit InverseKinematicsControl(loco::WholeBody& wholeBody, bool useAsTrackingController = false, bool useVelocityTracking = true,
                                    bool useDesiredTorsoState = false);

  //! Default Destructor.
  ~InverseKinematicsControl() override = default;

  /** Initializes the module
   * @param dt  time step
   * @return true, iff successful
   */
  bool initialize(double dt) override;

  /** Advances the module
   * @param dt  time step
   * @return    true, iff successful
   */
  bool advance(double dt) override;

  /** Loads parameters from an xml file
   *  @param handle tinyxml handle
   *  @return true, iff successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

 protected:
  /** Sets control mode of all limbs.
   *  @return true, iff successful
   */
  bool setControlModeForLimbs();

  /** Compute joint space control reference signals using inverse (differential) kinematics.
   *  @param dt  time step
   *  @return true, iff successful
   */
  bool computeControlReferences(double dt);

  /** Write desired joint reference signal of limb container.
   *  @return true, iff successful
   */
  bool setControlReferencesToLimbs();

  //! If false, this class is supposed to be used for impedance control on top of a WBC
  //! (only joint positions and velocities are overwritten).
  //! If true, this class is supposed to be used as tracking controller using PD control
  //! (additionally, joint modes and joint torques will be overwritten).
  bool useAsTrackingController_;

  //! If true, joint velocities are computed from reference end-effector velocities. If false, velocity is damped (joint velocities set to
  //! zero).
  bool useVelocityTracking_;

  //! If true, uses desired torso state, otherwise uses measured torso state.
  //! If useAsTrackingController_=true, this variable should be true aswell.
  //! Notice that, if set to true, desired torso state needs to be available!
  bool useDesiredTorsoState_;

  //! Control mode for limbs.
  loco::JointControlMode limbControlMode_;

  //! Control joint positions of a limbs (computed from inverse kinematics).
  std_utils::EnumArray<AD::ContactEnum, std::unique_ptr<basic_filters::FirstOrderFilter<Eigen::MatrixXd>>> desiredJointPositions_;

  //! Control joint velocities of a limbs (computed from inverse differential kinematics).
  std_utils::EnumArray<AD::ContactEnum, std::unique_ptr<basic_filters::FirstOrderFilter<Eigen::MatrixXd>>> desiredJointVelocities_;

  //! Control joint torques of a limbs (zero).
  std_utils::EnumArray<AD::ContactEnum, loco::JointTorques> desiredJointTorques_;
};

} /* namespace inverse_kinematics_control */
