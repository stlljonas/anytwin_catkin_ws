/*
 * AnymalModel.hpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Dario Bellicoso, Christian Gehring, Peter Fankhauser
 */

#pragma once

// anymal model
#include "anymal_model/AnymalInverseKinematics.hpp"
#include "anymal_model/AnymalParameters.hpp"
#include "anymal_model/AnymalState.hpp"
#include "anymal_model/LegConfigurations.hpp"
#include "anymal_model/LimitsAnymal.hpp"
#include "anymal_model/typedefs.hpp"

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// romo rbdl
#include <romo_rbdl/RobotModelRbdl.hpp>

// eigen
#include <Eigen/Core>

// stl
#include <memory>

namespace anymal_model {

class AnymalModel : public romo_rbdl::RobotModelRbdl<CAD, AnymalState> {
 private:
  using Base = romo_rbdl::RobotModelRbdl<CAD, AnymalState>;

 public:
  using typename Base::BodyEnum;
  using typename Base::BodyNodeEnum;
  using typename Base::BranchEnum;
  using typename Base::ContactEnum;
  using typename Base::ContactState;
  using typename Base::CoordinateFrameEnum;
  using typename Base::LimbEnum;
  using typename Base::RobotState;

 public:
  explicit AnymalModel(double dt = 0.0025);
  ~AnymalModel() override = default;

  /*! Initialize the anymal model by parsing a URDF
   * @returns true if the initialization is successful
   */
  bool initializeFromUrdfImpl(const std::string& urdfString, bool verbose) override;

  /*! Initialize the joint limits by parsing the string urdfDescription containing the URDF of the model
   * @returns true if the initialization is successful
   */
  bool initLimitsFromUrdfDescription(const std::string& urdfDescription);

  void setIsRealRobot(const bool isRealRobot);
  bool getIsRealRobot() const;

  /*******************
   * State accessors *
   *******************/
  void setState(const AnymalState& state, bool updatePosition, bool updateVelocity, bool updateAcceleration) override;
  /*******************/

  const JointAccelerations& getJointAccelerations() const;
  void setJointAccelerations(const JointAccelerations& jointAccelerations);

  const JointTorques& getJointTorques() const;
  void setJointTorques(const JointTorques& jointTorques);

  const Eigen::MatrixXd& getActuatorSelectionMatrix() const;

  /************************
   * Getters              *
   ************************/
  /*! Get the joint positions for a limb from the position vector by computing the analytic inverse kinematics.
   * @param legJoints                       the computed limb joints
   * @param positionBaseToFootInBaseFrame   the 3d position vector of the foot of the limb w.r.t. the main body
   * @param leg                             an integer representing the limb
   * @returns true if the inverse kinematics was successful
   */
  bool getLimbJointPositionsFromPositionBaseToFootInBaseFrame(Eigen::Vector3d& legJoints,
                                                              const Eigen::Vector3d& positionBaseToFootInBaseFrame, int leg);

  /*! Get the joint positions for a limb from the position vector by computing the analytic inverse kinematics.
   * @param legJoints                       the computed limb joints
   * @param positionBaseToFootInBaseFrame   the 3d position vector of the foot of the limb w.r.t. the main body
   * @param limb                            an enum representing the limb
   * @returns true if the inverse kinematics was successful
   */
  bool getLimbJointPositionsFromPositionBaseToFootInBaseFrame(Eigen::Vector3d& legJoints,
                                                              const Eigen::Vector3d& positionBaseToFootInBaseFrame, LimbEnum limb);

  /*! Get the kinematic parameters relative to each coordinate frame of the anymal.
   * @returns a const reference to an object containing the kinematic parameters of each body
   */
  const AnymalParameters& getParameters() const;

  double getMinHipToFootLength() const;
  double getMaxHipToFootLength() const;
  double getTimeStep() const override;

  const LegConfigurations& getLegConfigurations() const;
  void setLegConfigurations(const LegConfigurations& configurations);

  /*!
   *  @note Added by Christian
   */
  void getGeneralizedPositionsFromSymmetricJointPositions(Eigen::VectorXd& generalizedPositions, double leftForeHaa, double leftForeHfe,
                                                          double leftForeKfe);

  void addVariablesToLog(bool update, const std::string& nsPostfix = std::string{"/model/"});

  LimitsAnymal* getLimitsAnymal() { return dynamic_cast<LimitsAnymal*>(limitsPtr_.get()); }
  const LimitsAnymal* getLimitsAnymal() const { return dynamic_cast<LimitsAnymal*>(limitsPtr_.get()); }

 protected:
  double getHipToFootNorm(const Eigen::VectorXd& legJoints);
  double getThighToFootNorm(const Eigen::VectorXd& legJoints);
  double getLegGravityNorm(const Eigen::VectorXd& legJoints);

 protected:
  AnymalParameters params_;

  double minHipToFootLength_;
  double maxHipToFootLength_;
  double maxThighToFootLength_;

  LegConfigurations legConfigurations_;

  //! An instance of the class that implements the inverse kinematics for a 3DOF leg
  AnymalInverseKinematics inverseKinematics_;

  bool isRealRobot_;
  double timeStep_;

  Eigen::MatrixXd actuatorSelectionMatrix_;
};

}  // namespace anymal_model
