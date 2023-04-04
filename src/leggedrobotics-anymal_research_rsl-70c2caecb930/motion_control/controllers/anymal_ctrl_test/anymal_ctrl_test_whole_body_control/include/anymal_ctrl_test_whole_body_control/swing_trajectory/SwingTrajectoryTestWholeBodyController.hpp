/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Declaration of swing trajectory generator for WBC test controller
 * @date    Jul 15, 2019
 */
#pragma once

// loco_anymal
#include <loco_anymal/common/WholeBodyAnymal.hpp>

// robot_utils
#include <robot_utils/schedule/Schedule.hpp>

// parameter_handler
#include <parameter_handler/parameter_handler.hpp>

namespace anymal_ctrl_test_whole_body_control {

class SwingTrajectoryTestWholeBodyController : public loco::ModuleBase {
 public:
  explicit SwingTrajectoryTestWholeBodyController(loco_anymal::WholeBodyAnymal& wholeBody);
  ~SwingTrajectoryTestWholeBodyController() override = default;

  bool initialize(double /*dt*/) override { return true; };
  bool advance(double dt) override;
  bool loadParameters(const TiXmlHandle& /*handle*/) override;
  void computeTrajectory();
  void clearTrajectory();
  bool addParametersToHandler(const std::string& /*ns*/) override;
  bool isScheduleFinished() const;

 private:
  void setEEAndJointReferencesFromTrajectory(loco::LegBase* leg, double time);

  loco_anymal::WholeBodyAnymal& wholeBody_;

  double time_;
  robot_utils::Schedule<loco::Position> schedule_[4];
  loco::Position mirror_[4];
  loco::Position positionOffsetHipToFoot_;

  parameter_handler::Parameter<double> rampDuration_;
  parameter_handler::Parameter<double> sinusoidFrequency_;
  parameter_handler::Parameter<double> xAmplitude_;
  parameter_handler::Parameter<double> yAmplitude_;
  parameter_handler::Parameter<double> zAmplitude_;

  double sinusoidDuration_;

  bool isScheduleFinished_{false};
};

}  // namespace anymal_ctrl_test_whole_body_control
