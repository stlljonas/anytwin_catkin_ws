//  PIDController.hpp
//  Created by Philipp Leemann on 08.03.15.

#pragma once

#include "parameter_handler/parameter_handler.hpp"

namespace basic_controllers {

class PIDController {
 public:
  PIDController();
  PIDController(const double maxEffort, const double kp, const double ki = 0.0, const double kd = 0.0, const double kf = 0.0,
                const double maxIntegratorInput = std::numeric_limits<double>::max(),
                const double integratorSaturation = std::numeric_limits<double>::infinity());
  virtual ~PIDController() = default;

  void reset();
  double update(const double dt, const double desired, const double measured);
  double update(const double dt, const double desired, const double measured, const double desiredDerivative);
  double update(const double dt, const double desired, const double measured, const double desiredDerivative,
                const double measuredDerivative);

  void setGains(const double kp, const double ki = 0.0, const double kd = 0.0, const double kf = 0.0) {
    setKp(kp);
    setKi(ki);
    setKd(kd);
    setKf(kf);
  }

  inline void setKp(const double kp) { kp_.setValue(kp); }
  inline void setKi(const double ki) { ki_.setValue(ki); }
  inline void setKd(const double kd) { kd_.setValue(kd); }
  inline void setKf(const double kf) { kf_.setValue(kf); }
  inline void setMaxEffort(const double eff) { maxEffort_.setValue(eff); }
  inline void setMaxIntegratorInput(const double maxIntInput) { maxIntegratorInput_.setValue(maxIntInput); }
  inline void setIntegratorSaturation(const double integratorSaturation) { integratorSaturation_.setValue(integratorSaturation); }

  inline double getKp() const { return kp_.getValue(); }
  inline double getKi() const { return ki_.getValue(); }
  inline double getKd() const { return kd_.getValue(); }
  inline double getKf() const { return kf_.getValue(); }
  inline double getMaxEffort() const { return maxEffort_.getValue(); }
  inline double getMaxIntegratorInput() const { return maxIntegratorInput_.getValue(); }
  inline double getIntegratorSaturation() const { return integratorSaturation_.getValue(); }

  bool addParametersToHandler(const std::string& pidName);

 protected:
  double previousMeasured_, integral_;
  parameter_handler::Parameter<double> maxEffort_;
  parameter_handler::Parameter<double> kp_;
  parameter_handler::Parameter<double> ki_;
  parameter_handler::Parameter<double> kd_;
  parameter_handler::Parameter<double> kf_;
  parameter_handler::Parameter<double> maxIntegratorInput_;
  parameter_handler::Parameter<double> integratorSaturation_;
};

}  // namespace basic_controllers
