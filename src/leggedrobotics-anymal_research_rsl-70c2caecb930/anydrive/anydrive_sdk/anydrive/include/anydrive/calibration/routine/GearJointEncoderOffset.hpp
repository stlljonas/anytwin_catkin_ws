#pragma once

#include <fstream>
#include <iostream>

#include "anydrive/calibration/routine/CalibrationBase.hpp"

namespace anydrive {
namespace calibration {
namespace routine {

class GearJointEncoderOffset : public CalibrationBase {
 public:
  class Measurement {
   public:
    int32_t gearPositionTicks_ = 0;
    int32_t jointPositionTicks_ = 0;
    int32_t encoderOffsetMeasurement_ = 0;
    double temperature_ = 0.0;

    Measurement(int32_t gearPositionTicks, int32_t jointPositionTicks, int32_t encoderOffsetMeasurement, double temperature)
        : gearPositionTicks_(gearPositionTicks),
          jointPositionTicks_(jointPositionTicks),
          encoderOffsetMeasurement_(encoderOffsetMeasurement),
          temperature_(temperature) {}
  };

  using MeasurementSeries = std::vector<Measurement>;
  using Optimizer = OptimizationNelderMead<5>;
  using Vector = typename Optimizer::Vector;

 protected:
  const double motorVelocityAbs_ = 0.0;
  const uint64_t turns_ = 0;
  const std::string filenameTrunk_;

  const unsigned int n_ = 2;
  std::vector<double> multipliers_ = {-1.0, 1.0};
  std::vector<std::string> names_ = {"negative", "positive"};
  std::vector<MeasurementSeries> measurementSeriesVector_;

  // The calibration parameters are:
  // 0: Constant offset [ticks]
  // 1: Amplitude of the first sinusoid with period 2*pi [ticks]
  // 2: Phaseshift of the first sinusoid with period 2*pi [rad]
  // 3: Amplitude of the second sinusoid with period pi [ticks]
  // 4: Phaseshift of the second sinusoid with period pi [rad]
  Vector params_;

 public:
  GearJointEncoderOffset(const AnydrivePtr& anydrive, const double motorVelocityAbs, const uint64_t turns);
  ~GearJointEncoderOffset() override = default;

  const Vector& getParams();

 protected:
  bool collectData() override;
  bool postProcessData() override;

  static int32_t getEncoderOffsetMeasurement(const int32_t gearPositionTicks, const int32_t jointPositionTicks);
  bool fitFunction(Vector& params) const;
  void getInitializationParams(Vector& params0) const;
  static void makeParamsUnique(Vector& params);
  static std::string paramsToString(const Vector& params);
  double getEncoderOffsetApproximation(const Vector& params, const double jointPositionTicks) const;
  double getError(const Vector& params) const;

  std::string getFilenameTrunk() const;
  void toFiles(const Vector& params) const;
  void fromFiles();
};

}  // namespace routine
}  // namespace calibration
}  // namespace anydrive
