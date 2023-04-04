/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Declaration of WBC test controller class
 * @date    Jul 12, 2019
 */

#pragma once

#include <map>
#include <memory>

#include <anymal_motion_control/AnymalController.hpp>
#include <loco/common/ParameterSet.hpp>
#include <loco/common/TerrainModelBase.hpp>
#include <loco/heading_generation/HeadingGenerator.hpp>
#include <loco/locomotion_controller/LocomotionControllerModules.hpp>
#include <loco/motion_control/MotionControllerBase.hpp>
#include <loco/terrain_perception/TerrainPerceptionBase.hpp>
#include <loco_anymal/common/CommandTranslator.hpp>
#include <loco_anymal/common/LegsAnymal.hpp>
#include <loco_anymal/common/WholeBodyAnymal.hpp>
#include <loco_anymal/typedefs.hpp>
#include <message_logger/message_logger.hpp>
#include <parameter_handler/parameter_handler.hpp>

#include "anymal_ctrl_test_whole_body_control/mission_control/MissionControlTestWholeBodyController.hpp"
#include "anymal_ctrl_test_whole_body_control/swing_trajectory/SwingTrajectoryTestWholeBodyController.hpp"
#include "anymal_ctrl_test_whole_body_control/torso_control/TorsoControlTestWholeBodyController.hpp"

namespace anymal_ctrl_test_whole_body_control {

//! Test Whole Body Control controller
/*! This controller allows to test the WBC stand-alone on ANYmal
 */
class TestWholeBodyController : public anymal_motion_control::AnymalController {
 public:
  using Base = anymal_motion_control::AnymalController;
  using Mode = MissionControlTestWholeBodyController::LocomotionMode;
  using AD = anymal_description::AnymalDescription;

  TestWholeBodyController() = default;
  ~TestWholeBodyController() override = default;

  bool create() override;
  bool initialize() override;
  bool advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) override;
  bool reset() override;
  bool stop() override;
  bool preStop() override { return true; }

 protected:
  using JointErrors = Eigen::Matrix<double, AD::getJointsDimension(), 1>;
  using EndEffectorErrors = Eigen::Matrix<double, AD::getNumTranslationalDof(), 1>;
  using EndEffectorsErrors = Eigen::Matrix<double, AD::getNumTranslationalDof(), AD::getNumLegs()>;
  using BaseErrors = Eigen::Matrix<double, AD::getNumTranslationalDof(), 1>;

  enum class ControlLevel { POSITION, VELOCITY, ACCELERATION };

  bool loadParameterSet();
  bool isRobotStateValid();

  anymal_motion_control::SwitchResult goToReferenceType(anymal_motion_control::ReferenceType referenceType) override;
  void goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) override;

  void enableStateEstimatorCheck();
  void disableStateEstimatorCheck();
  virtual bool resetStateEstimator();

  JointErrors getJointsRmsError(ControlLevel controlLevel);
  TestWholeBodyController::EndEffectorErrors getEndEffectorRmsError(TestWholeBodyController::ControlLevel controlLevel,
                                                                    unsigned int limbId);
  BaseErrors getBaseRmsError(ControlLevel controlLevel);

 private:
  void logTrackingErrors();
  void clearTrackingErrors();

  // Accumulated errors of joint motion tracking at different levels (sum of squares)
  std::unordered_map<ControlLevel, JointErrors> jointTrackingErrors_;

  // Accumulated errors of ee motion tracking at different levels (sum of squares)
  std::unordered_map<ControlLevel, EndEffectorsErrors> eeTrackingErrors_;

  // Accumulated errors of base motion tracking at different levels (sum of squares)
  std::unordered_map<ControlLevel, BaseErrors> baseTrackingErrors_;

  // Number of error samples for computing RMS
  unsigned int numOfErrorSamples_ = 0;

  // legs
  std::unique_ptr<loco_anymal::LegsAnymal> legs_;

  // body
  std::unique_ptr<loco_anymal::TorsoAnymal> torso_;
  std::unique_ptr<loco_anymal::WholeBodyAnymal> wholeBody_;

  // terrain
  std::unique_ptr<loco::TerrainModelBase> terrainModel_;
  std::unique_ptr<loco::TerrainPerceptionBase> terrainPerception_;

  // strategy
  std::unique_ptr<TorsoControlTestWholeBodyController> torsoControl_;
  std::unique_ptr<loco::MotionControllerBase> motionControl_;
  std::unique_ptr<loco::ParameterSet> parameterSet_;
  std::unique_ptr<SwingTrajectoryTestWholeBodyController> swingTrajectoryGenerator_;
  std::unique_ptr<loco::HeadingGenerator> headingGenerator_;

  // external control/tuning
  std::unique_ptr<MissionControlTestWholeBodyController> missionController_;
  std::unique_ptr<loco::LocomotionControllerModules> locomotionController_;
  std::unique_ptr<loco_anymal::CommandTranslator> commandTranslator_;
};

} /* namespace anymal_ctrl_test_whole_body_control */
