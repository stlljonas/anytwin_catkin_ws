/**
 * @authors     Stephane Caron
 * @affiliation ANYbotics
 * @brief       Performance monitor for model-based controllers.
 */

// signal_logger
#include <signal_logger/signal_logger.hpp>

// loco_perf
#include "loco_perf/PerformanceMonitor.hpp"

namespace loco_perf {

bool PerformanceMonitor::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle performanceMonitor(handle);
  if (!tinyxml_tools::getChildHandle(performanceMonitor, handle, "PerformanceMonitor")) {
    return false;
  }

  TiXmlHandle twistTracking(handle);
  if (tinyxml_tools::getChildHandle(twistTracking, performanceMonitor, "TwistTracking")) {
    if (tinyxml_tools::loadParameter(timeConstant_, twistTracking, "time_constant", DEFAULT_FILTER_TIME_CONSTANT)) {
      twistTrackingErrorFilter_.setFilterParameters(dt_, timeConstant_);
      twistTrackingSquaredErrorFilter_.setFilterParameters(dt_, timeConstant_);
    }
  }
  return true;
}

bool PerformanceMonitor::initialize(double dt) {
  dt_ = dt;
  twistTrackingErrorAverage_.setZero();
  twistTrackingErrorFilter_.reset();
  twistTrackingError_.setZero();
  twistTrackingSquaredErrorFilter_.reset();
  return true;
}

inline Vector6d squareVector(const Vector6d& v) {
  return v.cwiseProduct(v);
}

bool PerformanceMonitor::advance(double /* dt */) {
  // Instantaneous torso velocity tracking error
  const auto& desiredTwistInControlFrame = missionController_.getDesiredBaseTwistInControlFrame();
  const auto& measuredAngularVelocityInControlFrame = torso_.getMeasuredState().inControlFrame().getAngularVelocityBaseInControlFrame();
  const auto& measuredLinearVelocityInControlFrame = torso_.getMeasuredState().inControlFrame().getLinearVelocityBaseInControlFrame();
  loco::LinearVelocity linearVelocityError = measuredLinearVelocityInControlFrame - desiredTwistInControlFrame.getTranslationalVelocity();
  loco::LocalAngularVelocity angularVelocityError =
      measuredAngularVelocityInControlFrame - desiredTwistInControlFrame.getRotationalVelocity();
  twistTrackingError_ = loco::Twist(linearVelocityError, angularVelocityError);

  // Average of torso velocity tracking error
  twistTrackingErrorFilter_.advance(twistTrackingError_.getVector());
  Vector6d filteredError = twistTrackingErrorFilter_.getFilteredValue();
  twistTrackingErrorAverage_.setVector(filteredError);

  // Variance of torso velocity tracking error
  twistTrackingSquaredErrorFilter_.advance(squareVector(twistTrackingError_.getVector()));
  Vector6d filteredSquaredError = twistTrackingSquaredErrorFilter_.getFilteredValue();
  Vector6d twistTrackingErrorVariance = filteredSquaredError - squareVector(filteredError);
  twistTrackingErrorStdDev_.setVector(twistTrackingErrorVariance.cwiseSqrt());
  return true;
}

bool PerformanceMonitor::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(timeConstant_, "timeConstant", ns, "s");
  signal_logger::add(twistTrackingErrorAverage_, "twistTrackingErrorAverage", ns);
  signal_logger::add(twistTrackingErrorStdDev_, "twistTrackingErrorStdDev", ns);
  signal_logger::add(twistTrackingError_, "twistTrackingError", ns);
  return true;
}

}  // namespace loco_perf
