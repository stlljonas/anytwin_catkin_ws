/*
 * WholeBodyController.hpp
 *
 *  Created on: Jun 5, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// stl
#include <vector>

// hierarchical optimization
#include <hierarchical_optimization/HierarchicalOptimizationBase.hpp>

// loco
#include <loco/motion_control/MotionControllerBase.hpp>
#include <loco/common/TerrainModelBase.hpp>
#include <loco/common/WholeBody.hpp>

// whole body control
#include <whole_body_control/TaskManager.hpp>
#include "whole_body_control_romo/typedefs.hpp"
#include "whole_body_control_romo/TaskLoaderRomo.hpp"
#include "whole_body_control_romo/support_jacobian/SupportJacobianRomo.hpp"

// tasks
#include "whole_body_control_romo/tasks/EoMTask.hpp"
#include "whole_body_control_romo/tasks/BaseMotionTask.hpp"
#include "whole_body_control_romo/tasks/BodyMotionTask.hpp"
#include "whole_body_control_romo/tasks/ContactTask.hpp"
#include "whole_body_control_romo/tasks/DistributeAccelerationsTask.hpp"
#include "whole_body_control_romo/tasks/DistributeContactForcesTask.hpp"
#include "whole_body_control_romo/tasks/EndEffectorMotionTask.hpp"
#include "whole_body_control_romo/tasks/EoMTask.hpp"
#include "whole_body_control_romo/tasks/FrictionPyramidTask.hpp"
#include "whole_body_control_romo/tasks/JointAccelerationTask.hpp"
#include "whole_body_control_romo/tasks/JointFreezeTask.hpp"
#include "whole_body_control_romo/tasks/JointPositionTask.hpp"
#include "whole_body_control_romo/tasks/JointPositionLimitsTask.hpp"
#include "whole_body_control_romo/tasks/JointTorqueLimitsTask.hpp"
#include "whole_body_control_romo/tasks/JointVelocityTask.hpp"
#include "whole_body_control_romo/tasks/JointVelocityLimitsTask.hpp"
#include "whole_body_control_romo/tasks/ContactWrenchTask.hpp"
#include "whole_body_control_romo/tasks/JointPositionVelocityTask.hpp"

// model
#include <romo/RobotModel.hpp>

// parameter handler
#include <parameter_handler/parameter_handler.hpp>

// utils
#include <std_utils/std_utils.hpp>

// pid control
#include <basic_controllers/PIDGains.hpp>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// type traits
#include <type_traits>

namespace whole_body_control_romo {

template<typename ConcreteDescription_, typename RobotState_>
class WholeBodyController : public loco::MotionControllerBase {
 private:
  using Base = loco::MotionControllerBase;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Robot description definitions.
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = typename RobotModel::RD;
  using ContactEnum = typename RD::ContactEnum;
  using BranchEnum = typename RD::BranchEnum;
  using LimbEnum = typename RD::LimbEnum;
  using BodyEnum = typename RD::BodyEnum;
  using BodyNodeEnum = typename RD::BodyNodeEnum;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;
  using ActuatorEnum = typename RD::ActuatorEnum;
  using ActuatorNodeEnum = typename RD::ActuatorNodeEnum;

  // Tasks
  using TaskLoader = TaskLoaderRomo<ConcreteDescription_, RobotState_>;
  using TaskManager = whole_body_control::TaskManager;
  using BaseTranslationTaskType = BaseTranslationTask<ConcreteDescription_, RobotState_>;
  using BaseRotationTaskType = BaseRotationTask<ConcreteDescription_, RobotState_>;
  using BodyTranslationTaskType = BodyTranslationTask<ConcreteDescription_, RobotState_>;
  using BodyRotationTaskType = BodyRotationTask<ConcreteDescription_, RobotState_>;
  using ContactTaskType = ContactTask<ConcreteDescription_, RobotState_>;
  using DistributeAccelerationsTaskType = DistributeAccelerationsTask<ConcreteDescription_, RobotState_>;
  using DistributeContactForcesTaskType = DistributeContactForcesTask<ConcreteDescription_, RobotState_>;
  using EndEffectorTranslationTaskType = EndEffectorTranslationTask<ConcreteDescription_, RobotState_>;
  using EndEffectorRotationTaskType = EndEffectorRotationTask<ConcreteDescription_, RobotState_>;
  using EoMTaskType = EoMTask<ConcreteDescription_, RobotState_>;
  using FrictionPyramidTaskType = FrictionPyramidTask<ConcreteDescription_, RobotState_>;
  using JointAccelerationTaskType = JointAccelerationTask<ConcreteDescription_, RobotState_>;
  using JointFreezeTaskType = JointFreezeTask<ConcreteDescription_, RobotState_>;
  using JointPositionTaskType = JointPositionTask<ConcreteDescription_, RobotState_>;
  using JointPositionLimitsTaskType = JointPositionLimitsTask<ConcreteDescription_, RobotState_>;
  using JointTorqueLimitsTaskType = JointTorqueLimitsTask<ConcreteDescription_, RobotState_>;
  using JointVelocityTaskType = JointVelocityTask<ConcreteDescription_, RobotState_>;
  using JointVelocityLimitsTaskType = JointVelocityLimitsTask<ConcreteDescription_, RobotState_>;
  using ContactWrenchTaskType = ContactWrenchTask<ConcreteDescription_, RobotState_>;
  using JointPositionVelocityTaskType = JointPositionVelocityTask<ConcreteDescription_, RobotState_>;

  // Support Jacobian definitions.
  using ConcreteSupportJacobian = SupportJacobianRomo<ConcreteDescription_, RobotState_>;
  using ContactFlags = typename ConcreteSupportJacobian::ContactFlags;

  using EnumArrayDouble = std_utils::EnumArray<LimbEnum, double>;
  using EnumArrayBool = std_utils::EnumArray<LimbEnum, bool>;
  using PIDGains3D = basic_controllers::PIDGains3D;

  WholeBodyController(
      loco::WholeBody& wholeBody,
      const RobotModel& model,
      loco::TerrainModelBase& terrainModel);
  ~WholeBodyController() override = default;

  /*!
     * Initialize the motion tracker
     * @return true if successful
     */
  bool initialize(double dt) override;

  /*!
   * Computes the joint torques from the desired base pose.
   * @return true if successful
   */
  bool advance(double dt) override;

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*!
   * Adds parameters to the parameter handler
   * @param  ns, namespace  of the parameters
   * @return true, iff successful
   */
  bool addParametersToHandler(const std::string & ns) override;

  /*!
   * Removes parameters from the parameter handler
   * @return true, iff successful
   */
  bool removeParametersFromHandler() override;

  /*!
   * Add data to logger (optional).
   * @return true if successful
   */
  bool addVariablesToLog(const std::string& ns) const override;

  /*! Sets the parameters to the interpolated ones between motionController1 and controller2.
   * @param motionController1     If the interpolation parameter is 0, then the parameter set is equal to the one of motionController1.
   * @param motionController2     If the interpolation parameter is 1, then the parameter set is equal to the one of motionController2.
   * @param t                     interpolation parameter in [0, 1]
   * @return                      true if successful
   */
  bool setToInterpolated(const loco::MotionControllerBase& motionController1, const loco::MotionControllerBase& motionController2, double t) override;

  /*! Switch task setup
   * @param newSetupName         The name of the task setup to switch to
   * @return                     true if successful
   */
  bool switchTaskSetup(const std::string& newSetupName);

 protected:
  virtual bool addOptimizationTasks();
  bool addOptimizationTaskDistributeTorques(int priority);
  bool addOptimizationTaskDistributeTangentialContactForces(int priority);

  void computeLegLoad(
      loco::LegBase& leg,
      const double startRampingDownAtStancePhase,
      const double endRampingUpAtStancePhase,
      const double minLegLoad = 0.0);

  virtual void updateContactFlags();

  virtual bool setControlModeForLimbs();

  virtual bool setControlReferencesToLimbs(const double dt);
  virtual bool setOptimizedReferences(const double dt);

  const RobotModel& model_;
  loco::TerrainModelBase& terrainModel_;
  hopt::HierarchicalOptimizationBasePtr hierarchicalOptimization_;

  Eigen::VectorXd ddqDes_;
  Eigen::VectorXd tauDes_;
  Eigen::VectorXd lambdaDes_;
  ContactFlags contactFlags_;

  // helper selection matrices
  ConcreteSupportJacobian supportJacobian_;

  enum class OptimizationMethod : unsigned int {
    NullspaceProjection = 0,
    QPCascade
  };
  OptimizationMethod optimizationMethod_;
  double dt_;

  parameter_handler::Parameter<double> startRampingDownAtStancePhase_;
  parameter_handler::Parameter<double> endRampingUpAtStancePhase_;

  std_utils::HighResolutionClockTimer timer_;

  EnumArrayDouble timeSinceSupport_;
  EnumArrayDouble legLoad_;
  bool adaptLegLoad_;

  Eigen::VectorXd optimalSolution_;
  unsigned int optimizationFailureCount_;
  bool didFail_;

  loco::ControlMode limbSupportControlMode_;
  loco::ControlMode limbNonSupportControlMode_;

  unsigned int solutionSpaceDimension_;

  //! If true, the reference joint velocities will be computed by integrating the optimized accelerations.
  bool computeJointVelocities_;

  //! Assembly of tasks
  WholeBodyStateRomo<ConcreteDescription_, RobotState_> wholeBodyState_;
  TaskManager taskManager_;

};

} /* namespace whole_body_control_romo */
