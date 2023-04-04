/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Declaration of mission controller for WBC test controller
 * @date    Jul 12, 2019
 */

#pragma once

#include <condition_variable>
#include <mutex>

// loco
#include <loco/common/torso/TorsoBase.hpp>
#include <loco/mission_control/MissionControlBase.hpp>
#include <loco/mission_control/MissionControlSpeedFilter.hpp>

// loco_anymal
#include <loco_anymal/motion_control/WholeBodyController.hpp>

// anymal_ctrl_test_whole_body_control
#include "anymal_ctrl_test_whole_body_control/swing_trajectory/SwingTrajectoryTestWholeBodyController.hpp"
#include "anymal_ctrl_test_whole_body_control/torso_control/TorsoControlTestWholeBodyController.hpp"

namespace anymal_ctrl_test_whole_body_control {

class MissionControlTestWholeBodyController : public loco::MissionControlBase {
 public:
  MissionControlTestWholeBodyController(loco::Pose* desRobotPose, loco::TorsoBase& torso,
                                        SwingTrajectoryTestWholeBodyController& swingTrajectory,
                                        TorsoControlTestWholeBodyController& torsoController,
                                        loco_anymal::WholeBodyController& wholeBodyController);
  ~MissionControlTestWholeBodyController() override = default;

  enum LocomotionMode {  // clang-format off
        ModeBaseMotion = 0,
        ModeEEMotion,
        ModeJointMotion,
        ModeUndefined };  // clang-format on

  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;
  const loco::Twist& getDesiredBaseTwistInControlFrame() const override;

  const loco::Twist& getMaximumBaseTwistInControlFrame() const override;
  const loco::Pose& getMinimalPoseOffset() const override;
  const loco::Pose& getMaximalPoseOffset() const override;

  // In joint motion mode, the legs are swung by tracking a joint-space
  // trajectory
  bool executeJointMotion();
  // In end-effector motion mode, the legs are swung by tracking a Cartesian
  // trajectory
  bool executeEEMotion();
  // In base motion mode, the robot tracks a base trajectory
  bool executeBaseMotion();
  // In freeze mode, the joints are blocked (freeze)
  bool goToFreeze();

  const std::string& getCurrentLocomotionMode() const;
  const std::string& getCurrentLocomotionState() const;
  LocomotionMode getLocomotionModeFromName(const std::string& name) const;
  bool stop() override;

 private:
  bool switchToStand();
  bool switchToSwing();
  bool switchToFreeze();

  inline void waitForTorsoScheduleToFinish();
  inline void waitForSwingScheduleToFinish();

  enum LocomotionState {  // clang-format off
      Freeze = 0,
      Standing,
      Swinging };  // clang-format on

  loco::Pose* desRobotPose_;
  loco::TorsoBase& torso_;
  SwingTrajectoryTestWholeBodyController& swingTrajectory_;
  TorsoControlTestWholeBodyController& torsoControl_;
  loco_anymal::WholeBodyController& wholeBodyController_;
  loco::MissionControlSpeedFilter speedFilter_;

  LocomotionState locomotionState_;
  LocomotionMode locomotionMode_;
  std::map<LocomotionMode, std::string> modeNameMap_ = {{LocomotionMode::ModeBaseMotion, "base_motion"},
                                                        {LocomotionMode::ModeEEMotion, "ee_motion"},
                                                        {LocomotionMode::ModeJointMotion, "joint_motion"},
                                                        {LocomotionMode::ModeUndefined, "undefined"}};
  std::map<LocomotionState, std::string> stateNameMap_ = {
      {LocomotionState::Standing, "standing"}, {LocomotionState::Freeze, "freeze"}, {LocomotionState::Swinging, "swinging"}};

  std::mutex advanceMutex_;
  std::condition_variable advanceCv_;
};

}  // namespace anymal_ctrl_test_whole_body_control
