/**
 * @authors     Stephane Caron
 * @affiliation ANYbotics
 * @brief       Performance monitor for model-based controllers.
 */

#pragma once

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// basic filters
#include <basic_filters/ExponentialMovingAverage.hpp>

// loco
#include <loco/common/torso/TorsoBase.hpp>
#include <loco/mission_control/MissionControlBase.hpp>

namespace loco_perf {

constexpr double DEFAULT_CONTROL_TIME_STEP = 0.0025;  // [s]
constexpr double DEFAULT_FILTER_TIME_CONSTANT = 5.;   // [s]

using Vector6d = Eigen::Matrix<double, 6, 1>;

class PerformanceMonitor : public loco::ModuleBase {
 public:
  /*! Constructor.
   *
   * @param torso Torso properties.
   * @param missionController Mission controller.
   */
  PerformanceMonitor(const loco::TorsoBase& torso, const loco::MissionControlBase& missionController)
      : ModuleBase("perf_monitor"),
        dt_(DEFAULT_CONTROL_TIME_STEP),
        timeConstant_(DEFAULT_FILTER_TIME_CONSTANT),
        torso_(torso),
        missionController_(missionController),
        twistTrackingErrorFilter_(DEFAULT_CONTROL_TIME_STEP, DEFAULT_FILTER_TIME_CONSTANT, Vector6d::Zero()),
        twistTrackingSquaredErrorFilter_(DEFAULT_CONTROL_TIME_STEP, DEFAULT_FILTER_TIME_CONSTANT, Vector6d::Zero()) {}

  //! Default destructor.
  ~PerformanceMonitor() override = default;

  /*! Initialize monitor.
   *
   * @param dt  time step in [s]
   * @return    true iff no problem
   */
  bool initialize(double dt) override;

  /*! @brief Load parameters.
   *
   * @param handle XML handle
   * @return       true iff successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*! Advance monitor.
   *
   * @param dt  time step in [s]
   * @return    true, iff successful
   */
  bool advance(double dt) override;

  /*! Add variables to signal log.
   *
   * @param ns Namespace prefix.
   */
  bool addVariablesToLog(const std::string& ns) const override;

 public:  // OUTPUTS, const getters only
  //! Twist tracking error
  const loco::Twist& getTwistTrackingError() { return twistTrackingError_; }

  //! Average of twist tracking error
  const loco::Twist& getTwistTrackingErrorAverage() { return twistTrackingErrorAverage_; }

  //! Standard deviation of twist tracking error
  const loco::Twist& getTwistTrackingErrorStdDev() { return twistTrackingErrorStdDev_; }

 private:
  //! Controller time step
  double dt_;

  //! Time constant for EMA filters
  double timeConstant_;

  //! Robot torso placeholder
  const loco::TorsoBase& torso_;

  //! Mission controller
  const loco::MissionControlBase& missionController_;

  //! Torso velocity tracking error
  loco::Twist twistTrackingError_;

  //! Average of torso velocity tracking error
  loco::Twist twistTrackingErrorAverage_;

  //! Variance of torso velocity tracking error
  loco::Twist twistTrackingErrorStdDev_;

  //! Filter for the average of the torso velocity tracking error
  basic_filters::ExponentialMovingAverage<Vector6d> twistTrackingErrorFilter_;

  //! Filter for the average of the squared torso velocity tracking error
  basic_filters::ExponentialMovingAverage<Vector6d> twistTrackingSquaredErrorFilter_;
};

}  // namespace loco_perf
