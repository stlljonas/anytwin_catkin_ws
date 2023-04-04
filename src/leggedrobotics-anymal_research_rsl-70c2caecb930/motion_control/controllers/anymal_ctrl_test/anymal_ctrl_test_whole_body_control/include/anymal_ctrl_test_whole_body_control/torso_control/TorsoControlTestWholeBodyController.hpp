/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Declaration of torso trajectory generator for WBC test controller
 * @date    Jul 15, 2019
 */
#pragma once

#include <atomic>

// loco
#include <loco/common/TerrainModelBase.hpp>
#include <loco/common/torso/TorsoBase.hpp>

// robot_utils
#include <robot_utils/schedule/ProfileRamp.hpp>
#include <robot_utils/schedule/ProfileSinusoid.hpp>
#include <robot_utils/schedule/ProfileStep.hpp>
#include <robot_utils/schedule/Schedule.hpp>

// parameter_handler
#include <loco/heading_generation/HeadingGenerator.hpp>
#include <parameter_handler/parameter_handler.hpp>

namespace anymal_ctrl_test_whole_body_control {

class TorsoControlTestWholeBodyController : public loco::ModuleBase {
 public:
  TorsoControlTestWholeBodyController(loco::TorsoBase& torso, loco::HeadingGenerator& headingGenerator, loco::TerrainModelBase& terrain);
  ~TorsoControlTestWholeBodyController() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;
  bool addParametersToHandler(const std::string& ns) override;
  void computeTrajectory();
  void clearTrajectory();
  const loco::Position& getMinPosOffset() const;
  const loco::Position& getMaxPosOffset() const;
  void setMinPosOffset(const loco::Position& minPosOffset);
  void setMaxPosOffset(const loco::Position& maxPosOffset);
  bool isScheduleFinished() const;

 private:
  loco::RotationQuaternion getOrientationWorldToHeadingOnTerrainSurface(const loco::RotationQuaternion& orientationWorldToHeading) const;
  loco::Position getDefaultTorsoPositionInWorldFrame(double heightOverTerrain) const;

  loco::TorsoBase& torso_;
  loco::HeadingGenerator& headingGenerator_;
  loco::TerrainModelBase& terrain_;

  double time_;  // TODO(paco): use any_measurements::Time?
  robot_utils::Schedule<loco::Position> positionSchedule_;
  robot_utils::Schedule<loco::RotationQuaternion> orientationSchedule_;

  double restDuration_;
  parameter_handler::Parameter<double> defaultTorsoHeight_;
  parameter_handler::Parameter<double> rampDuration_;
  static constexpr double initialRampDuration_ = 3.0;
  loco::Position defaultTorsoPositionInWorldFrameAtStart_;

  loco::Position minPosOffset_;
  loco::Position maxPosOffset_;
  loco::EulerAnglesXyz minOrientOffset_;
  loco::EulerAnglesXyz maxOrientOffset_;

  std::atomic<bool> isScheduleFinished_{false};
};

}  // namespace anymal_ctrl_test_whole_body_control
