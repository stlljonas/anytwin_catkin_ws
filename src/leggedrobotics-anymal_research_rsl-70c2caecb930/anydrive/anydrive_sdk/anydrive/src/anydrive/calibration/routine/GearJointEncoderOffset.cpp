#include "anydrive/calibration/routine/GearJointEncoderOffset.hpp"

namespace anydrive {
namespace calibration {
namespace routine {

GearJointEncoderOffset::GearJointEncoderOffset(const AnydrivePtr& anydrive, const double motorVelocityAbs, const uint64_t turns)
    : CalibrationBase(anydrive), motorVelocityAbs_(motorVelocityAbs), turns_(turns), filenameTrunk_(getFilenameTrunk()) {
  assert(n_ == multipliers_.size());
  assert(n_ == names_.size());
}

const GearJointEncoderOffset::Vector& GearJointEncoderOffset::getParams() {
  return params_;
}

bool GearJointEncoderOffset::collectData() {
  if (stopRequested()) {
    return false;
  }

  bool readFromFiles = false;
  if (readFromFiles) {
    fromFiles();
  } else {
    uint32_t gearRatio = 0;
    if (!getGearRatio(gearRatio)) {
      return false;
    }

    // Note that the following duration is not accurately reached by the drive... instead of ca. 11.4s, it will spin for >15s.
    // However, this should not be a problem for the calibration.
    const double duration = 2.0 * M_PI * turns_ * static_cast<double>(gearRatio) / motorVelocityAbs_;

    for (unsigned int i = 0; i < n_; i++) {
      if (stopRequested()) {
        return false;
      }

      const double motorVelocity = multipliers_[i] * motorVelocityAbs_;
      const double durationOffset = 2.0f;  // Some time for the drive to spin up and reach constant velocity
      ANYDRIVE_INFO("Turning with motor v = " << motorVelocity << " rad/s for ca. " << turns_ << " turns (" << names_[i]
                                              << " measurement series).");
      stageCommandMotorVelocity(motorVelocity);  // The motor starts spinning
      if (!preemptableSleep(durationOffset)) {   // Wait for the velocity to stabilize
        return false;
      }
      anydrive_->startLog(names_[i]);
      if (!preemptableSleep(duration)) {  // Obtain the data while the drive spins
        return false;
      }
      anydrive_->stopLog();
      if (!preemptableSleep(durationOffset)) {  // Settle down.
        return false;
      }
      ANYDRIVE_INFO("Stopping.");
      stageCommandDisable();
      if (!preemptableSleep(1.0)) {
        return false;
      }

      ANYDRIVE_INFO("Reading logs.");
      const LogReading& log = anydrive_->getLog(names_[i]);
      MeasurementSeries measurementSeries;
      for (const auto& measurement : log.getData()) {
        const int32_t encoderOffsetMeasurement =
            getEncoderOffsetMeasurement(measurement.getState().getGearPositionTicks(), measurement.getState().getJointPositionTicks());
        measurementSeries.push_back(Measurement(measurement.getState().getGearPositionTicks(),
                                                measurement.getState().getJointPositionTicks(), encoderOffsetMeasurement,
                                                measurement.getState().getTemperature()));
      }
      measurementSeriesVector_.push_back(measurementSeries);
      ANYDRIVE_INFO("Read " << measurementSeries.size() << " measurements.");
    }
  }

  return true;
}

bool GearJointEncoderOffset::postProcessData() {
  ANYDRIVE_INFO("Encoder offset calibration: Fitting function.");
  bool success;
  params_ = Vector::Zero();
  success = fitFunction(params_);
  if (writeToFiles_) {
    toFiles(params_);
  }
  return success;
}

int32_t GearJointEncoderOffset::getEncoderOffsetMeasurement(const int32_t gearPositionTicks, const int32_t jointPositionTicks) {
  return gearPositionTicks - jointPositionTicks;
}

/*
 * This function fits the measured data onto the correction/calibration function.
 * It also performs sanity-checks to make sure no faulty data is being fitted/generated.
 * This could potentially brick a drive, as faulty compensation parameters create
 * faulty torques, which immediately leads to drive errors.
 * The params are only returned if everything succeeds. Otherwise, all params will be zero.
 */
bool GearJointEncoderOffset::fitFunction(Vector& params) const {
  Vector params0;
  getInitializationParams(params0);
  ANYDRIVE_INFO("Encoder offset calibration: Initialization parameters:");
  ANYDRIVE_INFO(paramsToString(params0));

  Optimizer optimizer;
  double error = 0;
  Vector tentativeParams;
  bool success =
      optimizer.optimize(error, tentativeParams, params0, std::bind(&GearJointEncoderOffset::getError, this, std::placeholders::_1), true);
  if (!success) {
    ANYDRIVE_ERROR("The optimizer could not fit the data. Aborting calibration.");
    return false;
  }

  ANYDRIVE_INFO("Encoder offset calibration: Optimized parameters before check:");
  ANYDRIVE_INFO(paramsToString(tentativeParams));

  makeParamsUnique(tentativeParams);

  ANYDRIVE_INFO("Encoder offset calibration: Optimized parameters:");
  ANYDRIVE_INFO(paramsToString(tentativeParams));
  ANYDRIVE_INFO("Encoder offset calibration: Optimization residual error: " << error);

  // Check, if the optimizer residual is 0.0... This usually means that something went very wrong, e.g.,
  // faulty or constant drive encoder readings.
  if (std::abs(error) < 1e-2) {
    ANYDRIVE_ERROR("The optimizer residual error is (too close to) zero. Something went wrong. Aborting calibration.");
    return false;
  }
  // Sanity-check the determined offset (params[0]). It must be within the encoder tick range.
  if (std::abs(tentativeParams[0]) > ticks_) {
    ANYDRIVE_ERROR("The determined gear/joint offset is too large and unfeasible. Aborting calibration.");
    return false;
  }
  params = tentativeParams;
  return true;
}

void GearJointEncoderOffset::getInitializationParams(Vector& params0) const {
  double averageOffset = 0;
  unsigned int numberOfMeasurements = 0;
  for (const auto& measurementSeries : measurementSeriesVector_) {
    for (const auto& measurement : measurementSeries) {
      averageOffset += measurement.encoderOffsetMeasurement_;
      numberOfMeasurements++;
    }
  }
  averageOffset /= numberOfMeasurements;

  params0[0] = averageOffset;
  params0[1] = 0.0;
  params0[2] = 0.0;
  params0[3] = 0.0;
  params0[4] = 0.0;
}

void GearJointEncoderOffset::makeParamsUnique(Vector& params) {
  for (unsigned int i = 1; i <= 3; i += 2) {
    if (params[i] < 0) {
      params[i] *= -1.0;
      params[i + 1] += M_PI;
    }
    params[i + 1] = wrapTwoPI(params[i + 1]);
  }
}

std::string GearJointEncoderOffset::paramsToString(const Vector& params) {
  std::stringstream ss;
  ss << params[0] << "," << params[1] << "," << params[2] << "," << params[3] << "," << params[4];
  return ss.str();
}

double GearJointEncoderOffset::getEncoderOffsetApproximation(const Vector& params, const double jointPositionTicks) const {
  return params[0] + params[1] * std::sin(static_cast<double>(omega_ * jointPositionTicks + params[2])) +
         params[3] * std::sin(static_cast<double>(2.0 * omega_ * jointPositionTicks + params[4]));
}

double GearJointEncoderOffset::getError(const Vector& params) const {
  double error = 0;
  for (const auto& measurementSeries : measurementSeriesVector_) {
    for (const auto& measurement : measurementSeries) {
      const double encoderOffsetApproximation = getEncoderOffsetApproximation(params, measurement.jointPositionTicks_);
      error += std::pow(static_cast<double>(measurement.encoderOffsetMeasurement_ - encoderOffsetApproximation), 2);
    }
  }
  return error;
}

std::string GearJointEncoderOffset::getFilenameTrunk() const {
  return "encoder_offset_" + std::to_string(motorVelocityAbs_) + "_" + std::to_string(turns_);
}

void GearJointEncoderOffset::toFiles(const Vector& params) const {
  for (unsigned int i = 0; i < n_; i++) {
    ANYDRIVE_INFO("Writing " << names_[i] << " measurement series to file.");
    const std::string filePath = filesFolder_ + "/" + filenameTrunk_ + "_" + names_[i] + ".txt";
    std::ofstream file;
    file.open(filePath);
    for (const auto& measurement : measurementSeriesVector_[i]) {
      file << measurement.gearPositionTicks_ << ",";
      file << measurement.jointPositionTicks_ << ",";
      file << measurement.temperature_ << ",";
      file << measurement.encoderOffsetMeasurement_ << ",";
      file << getEncoderOffsetApproximation(params, measurement.jointPositionTicks_) << ",";
      file << measurement.encoderOffsetMeasurement_ - getEncoderOffsetApproximation(params, measurement.jointPositionTicks_);
      file << std::endl;
    }
    file.close();
    ANYDRIVE_INFO("Wrote " << measurementSeriesVector_[i].size() << " measurements to file.");
  }
}

void GearJointEncoderOffset::fromFiles() {
  measurementSeriesVector_.clear();

  for (unsigned int i = 0; i < n_; i++) {
    ANYDRIVE_INFO("Reading " << names_[i] << " measurement series from file.");
    const std::string filePath = filesFolder_ + "/" + filenameTrunk_ + "_" + names_[i] + ".txt";
    std::ifstream file;
    file.open(filePath);
    if (!file.is_open()) {
      ANYDRIVE_ERROR("File '" << filePath << "' could not be opened.");
      continue;
    }

    MeasurementSeries measurementSeries;
    uint32_t gearPositionTicks = 0;
    uint32_t jointPositionTicks = 0;
    double temperature = 0;
    int64_t dummy1 = 0;
    double dummy2 = 0;
    double dummy3 = 0;
    while (file >> gearPositionTicks >> jointPositionTicks >> temperature >> dummy1 >> dummy2 >> dummy3) {
      measurementSeries.push_back(Measurement(gearPositionTicks, jointPositionTicks,
                                              getEncoderOffsetMeasurement(gearPositionTicks, jointPositionTicks), temperature));
    }
    measurementSeriesVector_.push_back(measurementSeries);
    file.close();
    ANYDRIVE_INFO("Read " << measurementSeries.size() << " measurements from file.");
  }
}

}  // namespace routine
}  // namespace calibration
}  // namespace anydrive
