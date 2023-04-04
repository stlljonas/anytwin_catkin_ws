/*!
 * @file     VirtualModelController.hpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso
 * @date     Aug 6, 2013
 * @brief
 */

#pragma once

#include <loco/contact_force_distribution/ContactForceDistributionInterface.hpp>
#include "loco/common/WholeBody.hpp"

// Motion Controller
#include "loco/motion_control/JacobianTranspose.hpp"
#include "loco/motion_control/MotionControllerBase.hpp"

// Contact force distribution
#include <Eigen/Core>
// Cout
#include <iostream>
#include "tinyxml.h"

#include <parameter_handler/parameter_handler.hpp>
#include "loco/common/typedefs.hpp"

namespace loco {

class VirtualModelController : public MotionControllerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Constructor.
   */
  VirtualModelController(WholeBody& wholeBody, ContactForceDistributionInterface& contactForceDistribution);
  /*!
   * Destructor.
   */
  ~VirtualModelController() override = default;

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*!
   * Add variables to log (optional).
   * @return true if successful.
   */
  bool addVariablesToLog(const std::string& ns) const override;

  bool addParametersToHandler(const std::string& ns) override;

  bool initialize(double dt) override;

  /*!
   * Computes the joint torques from the desired base pose.
   * @return true if successful.
   */
  bool advance(double dt) override;

  void print(std::ostream& out) const override;

  const Force& getDesiredVirtualForceInBaseFrame() const;
  const Torque& getDesiredVirtualTorqueInBaseFrame() const;
  void getDistributedVirtualForceAndTorqueInBaseFrame(Force& netForce, Torque& netTorque) const;

  const Eigen::Vector3d& getProportionalGainTranslation() const;
  const Eigen::Vector3d& getDerivativeGainTranslation() const;
  const Eigen::Vector3d& getIntegralGainTranslation() const;
  const Eigen::Vector3d& getFeedforwardGainTranslation() const;

  const Eigen::Vector3d& getProportionalGainRotation() const;
  const Eigen::Vector3d& getDerivativeGainRotation() const;
  const Eigen::Vector3d& getIntegralGainRotation() const;
  const Eigen::Vector3d& getFeedforwardGainRotation() const;

  void setProportionalGainTranslation(const Eigen::Vector3d& gains);
  void setDerivativeGainTranslation(const Eigen::Vector3d& gains);
  void setFeedforwardGainTranslation(const Eigen::Vector3d& gains);
  void setIntegralGainsTranslation(const Eigen::Vector3d& gains);

  void setProportionalGainRotation(const Eigen::Vector3d& gains);
  void setDerivativeGainRotation(const Eigen::Vector3d& gains);
  void setFeedforwardGainRotation(const Eigen::Vector3d& gains);
  void setIntegralGainsRotation(const Eigen::Vector3d& gains);

  void setGainsHeading(double kp, double kd, double ki, double kff);
  void setGainsLateral(double kp, double kd, double ki, double kff);
  void setGainsVertical(double kp, double kd, double ki, double kff);
  void setGainsRoll(double kp, double kd, double ki, double kff);
  void setGainsPitch(double kp, double kd, double ki, double kff);
  void setGainsYaw(double kp, double kd, double ki, double kff);

  void getGainsHeading(double& kp, double& kd, double& ki, double& kff);
  void getGainsLateral(double& kp, double& kd, double& ki, double& kff);
  void getGainsVertical(double& kp, double& kd, double& ki, double& kff);
  void getGainsRoll(double& kp, double& kd, double& ki, double& kff);
  void getGainsPitch(double& kp, double& kd, double& ki, double& kff);
  void getGainsYaw(double& kp, double& kd, double& ki, double& kff);

  double getGravityCompensationForcePercentage() const;
  void setGravityCompensationForcePercentage(double percentage);
  const ContactForceDistributionInterface& getContactForceDistribution() const;

  /*! Sets the parameters to the interpolated ones between motionController1 and controller2.
   * @param motionController1     If the interpolation parameter is 0, then the parameter set is equal to the one of motionController1.
   * @param motionController2     If the interpolation parameter is 1, then the parameter set is equal to the one of motionController2.
   * @param t                     interpolation parameter in [0, 1]
   * @return                      true if successful
   */
  bool setToInterpolated(const MotionControllerBase& motionController1, const MotionControllerBase& motionController2, double t) override;

  const Eigen::Vector3d& getIntegralGainTranslation();
  const Eigen::Vector3d& getIntegralGainRotation();
  const Force& getVirtualIntegralForceLimit();
  const Torque& getVirtualIntegralTorqueLimit();

  void resetIntegrator();
  void setIntegralGainsToZero();
  void setIsIntegratingTheErrors(bool isIntegratingTheErrors);

  const ControlMode& getSupportLegControlMode() const;
  const ControlMode& getNonSupportLegControlMode() const;

 protected:
  //! Distributes torso wrench over contact limbs
  ContactForceDistributionInterface& contactForceDistribution_;

  //! Controller that compute torques from desired contact forces
  JacobianTranspose jacobianTranspose_;

  //! Base position error in base frame.
  Position positionErrorInControlFrame_;

  //! Base orientation error vector (rotation vector) in base frame.
  Eigen::Vector3d orientationError_;

  //! Base linear velocity error in base frame.
  LinearVelocity linearVelocityErrorInControlFrame_;

  //! Base angular velocity error in base frame.
  LocalAngularVelocity angularVelocityErrorInControlFrame_;

  Force virtualIntegralForceInControlFrame_;
  Force virtualIntegralForceLimit_;
  Torque virtualIntegralTorqueInBaseFrame_;
  Torque virtualIntegralTorqueLimit_;

  //! Force on torso to compensate for gravity (in base frame).
  Force gravityCompensationForceInBaseFrame_;

  //! Torque on torso to compensate for gravity (in base frame).
  Torque gravityCompensationTorqueInBaseFrame_;

  //! A scaling factor applied to the gravity compensation terms.
  double gravityCompensationForcePercentage_;

  //! Desired virtual force on base in base frame (B_F_B^d).
  Force virtualForceInBaseFrame_;

  //! Desired virtual torque on base in base frame (B_T_B^d).
  Torque virtualTorqueInBaseFrame_;

  //! When this flag is true, an integrator action will be added to the controller.
  bool isIntegratingTheErrors_;

  Force offsetForceInBaseFrame_;
  Torque offsetTorqueInBaseFrame_;

  //! Proportional gain vector (k_p)  for translational error (force).
  parameter_handler::Parameter<Eigen::Vector3d> proportionalGainTranslation_;
  parameter_handler::Parameter<Eigen::Vector3d> derivativeGainTranslation_;
  parameter_handler::Parameter<Eigen::Vector3d> integralGainTranslation_;
  parameter_handler::Parameter<Eigen::Vector3d> feedforwardGainTranslation_;

  parameter_handler::Parameter<Eigen::Vector3d> proportionalGainRotation_;
  parameter_handler::Parameter<Eigen::Vector3d> derivativeGainRotation_;
  parameter_handler::Parameter<Eigen::Vector3d> integralGainRotation_;
  parameter_handler::Parameter<Eigen::Vector3d> feedforwardGainRotation_;

  loco::ControlMode supportLegControlMode_;
  loco::ControlMode nonSupportLegControlMode_;

 protected:
  /*!
   * Compute error between desired an actual robot pose and twist.
   * @return true if successful.
   */
  virtual bool computeError();

  /*!
   * Computes the gravity compensation force and torque.
   * @return true if successful.
   */
  virtual bool computeGravityCompensation();

  //! Compute virtual force based on the form:
  //! F = k_p*(q_d-q) + k_d*(q_dot_d-q_dot) + k_ff*(...).
  //! In base frame.
  virtual bool computeVirtualForce(double dt);

  //! Compute virtual torque based on the form:
  //! T = k_p*(q_d-q) + k_d*(q_dot_d-q_dot) + k_ff*(...).
  //! In base frame.
  virtual bool computeVirtualTorque(double dt);

  virtual bool setControlModeForLimbs();
};

} /* namespace loco */
