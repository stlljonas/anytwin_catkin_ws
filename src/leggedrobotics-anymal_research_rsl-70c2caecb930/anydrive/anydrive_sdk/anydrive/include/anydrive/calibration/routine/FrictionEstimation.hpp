#pragma once

#include <fstream>
#include <iostream>

#include "anydrive/calibration/routine/CalibrationBase.hpp"

namespace anydrive {
namespace calibration {
namespace routine {

class FrictionEstimation : public CalibrationBase {
 public:
  class Measurement {
   public:
    int32_t jointPositionTicks_ = 0;
    double jointVelocity_ = 0.0;
    double current_ = 0.0;
    double jointTorque_ = 0.0;
    double temperature_ = 0.0;

    Measurement(int32_t jointPositionTicks, double jointVelocity, double current, double jointTorque, double temperature)
        : jointPositionTicks_(jointPositionTicks),
          jointVelocity_(jointVelocity),
          current_(current),
          jointTorque_(jointTorque),
          temperature_(temperature) {}
  };

  using MeasurementSeries = std::vector<Measurement>;
  using Optimizer = OptimizationNelderMead<3>;
  using Vector = typename Optimizer::Vector;

 protected:
  const double duration_ = 0.0;
  const std::string filenameTrunk_;

  // Motor velocities to gather friction data.
  // Note: The velocities within [-breakAwayTorqueBandRpm_, breakAwayTorqueBandRpm_] are
  // only for informational purpose and will not be used for the friction evaluation.
  // The following array nonetheless contains a large span, should the breakAwayTorqueBandRpm_ be changed in future.
  // Note: The max. motor velocity of ANYdrive 3.0.x is approx. 350...380 rad/s.
  std::vector<double> motorVelocities_ = {-100.0, -125.0, -150.0, -175.0, -200.0, -225.0, -250.0, -275.0, -300.0, -325.0, -350.0,
                                          100.0,  125.0,  150.0,  175.0,  200.0,  225.0,  250.0,  275.0,  300.0,  325.0,  350.0};
  std::vector<std::string> names_;
  std::vector<MeasurementSeries> measurementSeriesVector_;

  const double breakAwayTorqueBandRpm_ = 20.0;
  const bool skipJointVelocitiesInsideBand_ = true;

  // The calibration parameters are:
  // 0: Break away joint torque [Nm]
  // 1: Negative viscous friction coefficient [Nm/rpm]
  // 2: Positive viscous friction coefficient [Nm/rpm]
  Vector params_;

 public:
  FrictionEstimation(const AnydrivePtr& anydrive, const double duration);
  ~FrictionEstimation() override = default;

  const Vector& getParams() const;
  double getBreakAwayTorqueBand() const;

 protected:
  bool collectData() override;
  bool postProcessData() override;

  bool fitFunction(Vector& params) const;
  void getInitializationParams(Vector& params0) const;
  static bool makeParamsUnique(Vector& params);
  static std::string paramsToString(const Vector& params);
  static double smoothSign(const double x, const double xBand);
  bool jointVelocityIsInsideBand(const double jointVelocity) const;
  double getFrictionEstimation(const Vector& params, const double jointVelocity) const;
  double getError(const Vector& params) const;

  std::string getFilenameTrunk() const;
  void toFiles(const Vector& params) const;
  void fromFiles();
};

}  // namespace routine
}  // namespace calibration
}  // namespace anydrive
