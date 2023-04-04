#pragma once

#include <fstream>
#include <iostream>

#include "anydrive/calibration/routine/CalibrationBase.hpp"

namespace anydrive {
namespace calibration {
namespace routine {

class SafeJointVelocity : public CalibrationBase {
 protected:
  class Measurement {
   public:
    int32_t jointPositionRawTicks_ = 0;
    double jointTorque_ = 0.0;
    double temperature_ = 0.0;

    Measurement(int32_t jointPositionRawTicks, double jointTorque, double temperature)
        : jointPositionRawTicks_(jointPositionRawTicks), jointTorque_(jointTorque), temperature_(temperature) {}
  };

  using MeasurementSeries = std::vector<Measurement>;
  using Optimizer = OptimizationNelderMead<3>;
  using Vector = typename Optimizer::Vector;

  const double jointVelocityAbs_ = 0.0;
  const uint64_t turns_ = 0;
  const std::string filenameTrunk_;

  const unsigned int n_ = 2;
  unsigned int i_ = 0;
  std::vector<double> multipliers_ = {-1.0, 1.0};
  std::vector<std::string> names_ = {"negative", "positive"};
  std::vector<MeasurementSeries> measurementSeriesVector_;

 public:
  SafeJointVelocity(const AnydrivePtr& anydrive, const double jointVelocityAbs, const uint64_t turns);
  ~SafeJointVelocity() override = default;

 protected:
  bool collectData() override;
  bool postProcessData() override;

  void fitFunction(Vector& params);
  void getInitializationParams(Vector& params0) const;
  static void makeParamsUnique(Vector& params);
  static std::string paramsToString(const Vector& params);
  double getGravityCompensation(const Vector& params, const double jointPositionTicks) const;
  double getError(const Vector& params) const;

  std::string getFilenameTrunk() const;
  void toFiles(const Vector& params) const;
  void fromFiles();
};

}  // namespace routine
}  // namespace calibration
}  // namespace anydrive
