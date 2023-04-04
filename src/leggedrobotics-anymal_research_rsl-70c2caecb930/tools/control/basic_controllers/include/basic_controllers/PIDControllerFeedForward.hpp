//  PIDController.hpp
//  Created by Philipp Leemann on Jan 2018

#pragma once

#include "parameter_handler/parameter_handler.hpp"

#include "basic_controllers/PIDController.hpp"

namespace basic_controllers {

class PIDControllerFeedForward : public PIDController {
 public:
  PIDControllerFeedForward();
  PIDControllerFeedForward(const double maxEffort, const double kp, const double ki = 0.0, const double kd = 0.0, const double kf = 0.0,
                           const double feedForwardGain = 0.0);
  ~PIDControllerFeedForward() override = default;

  double update(const double dt, const double desired, const double measured, const double feedForward);
  double update(const double dt, const double desired, const double measured, const double desiredDerivative, const double feedForward);
  double update(const double dt, const double desired, const double measured, const double desiredDerivative,
                const double measuredDerivative, const double feedForward);

  inline void setFeedForwardGain(const double gain) { feedForwardGain_.setValue(gain); }

  inline double getFeedForwardGain() const { return feedForwardGain_.getValue(); }

  bool addParametersToHandler(const std::string& pidName);

 protected:
  parameter_handler::Parameter<double> feedForwardGain_;
};

}  // namespace basic_controllers
