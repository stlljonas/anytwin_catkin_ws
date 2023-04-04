//  PIDController.cpp
//  Created by Philipp Leemann on Jan 2018

#include "basic_controllers/PIDControllerFeedForward.hpp"

namespace basic_controllers {

PIDControllerFeedForward::PIDControllerFeedForward() : PIDController(10.0, 1.0, 0.0, 0.0, 0.0) {}

PIDControllerFeedForward::PIDControllerFeedForward(const double maxEffort, const double kp, const double ki, const double kd,
                                                   const double kf, const double feedForwardGain)
    : PIDController(maxEffort, kp, ki, kd, kf),
      feedForwardGain_("Pid/FeedForwardGain", feedForwardGain, std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()) {}

double PIDControllerFeedForward::update(const double dt, const double desired, const double measured, const double feedForward) {
  const double measuredDerivative = (measured - previousMeasured_) / dt;
  return update(dt, desired, measured, 0.0, measuredDerivative, feedForward);
}

double PIDControllerFeedForward::update(const double dt, const double desired, const double measured, const double desiredDerivative,
                                        const double feedForward) {
  const double measuredDerivative = (measured - previousMeasured_) / dt;
  return update(dt, desired, measured, desiredDerivative, measuredDerivative, feedForward);
}

double PIDControllerFeedForward::update(const double dt, const double desired, const double measured, const double desiredDerivative,
                                        const double measuredDerivative, const double feedForward) {
  return PIDController::update(dt, desired, measured, desiredDerivative, measuredDerivative) + feedForwardGain_.getValue() * feedForward;
}

bool PIDControllerFeedForward::addParametersToHandler(const std::string& pidName) {
  parameter_handler::handler->addParam(pidName + std::string("/FeedForwardGain"), feedForwardGain_);
  return PIDController::addParametersToHandler(pidName);
}

}  // namespace basic_controllers
