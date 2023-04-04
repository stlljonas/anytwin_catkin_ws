#include "anydrive/calibration/routine/FrictionEstimation.hpp"

namespace anydrive {
namespace calibration {
namespace routine {

FrictionEstimation::FrictionEstimation(const AnydrivePtr& anydrive, const double duration)
    : CalibrationBase(anydrive), duration_(duration), filenameTrunk_(getFilenameTrunk()) {
  // Obtain the gear ratio:
  uint32_t gearRatio = 0;
  if (!getGearRatio(gearRatio)) ANYDRIVE_ERROR("Could not obtain gearbox ratio from device.")

  // Remove the motor velocities that lie inside the break-away friction band:
  if (skipJointVelocitiesInsideBand_) {
    for (auto it = motorVelocities_.begin(); it != motorVelocities_.end();) {
      if (jointVelocityIsInsideBand(*it / gearRatio)) {
        motorVelocities_.erase(it);
      } else {
        it++;
      }
    }
  }

  for (double& motorVelocity : motorVelocities_) {
    names_.push_back(std::to_string(motorVelocity));
  }
}

const FrictionEstimation::Vector& FrictionEstimation::getParams() const {
  return params_;
}

double FrictionEstimation::getBreakAwayTorqueBand() const {
  return breakAwayTorqueBandRpm_;
}

bool FrictionEstimation::collectData() {
  if (stopRequested()) {
    return false;
  }

  bool readFromFiles = false;
  if (readFromFiles) {
    fromFiles();
  } else {
    for (unsigned int i = 0; i < motorVelocities_.size(); i++) {
      if (stopRequested()) {
        return false;
      }

      ANYDRIVE_INFO("Turning with motor v = " << motorVelocities_[i] << " rad/s for " << duration_ << " s (" << names_[i]
                                              << " measurement series).");
      stageCommandMotorVelocity(motorVelocities_[i]);
      if (!preemptableSleep(1.0)) {
        return false;
      }
      anydrive_->startLog(names_[i]);
      if (!preemptableSleep(duration_)) {
        return false;
      }
      anydrive_->stopLog();
      if (!preemptableSleep(1.0)) {
        return false;
      }

      ANYDRIVE_INFO("Reading logs.");
      const LogReading& log = anydrive_->getLog(names_[i]);
      MeasurementSeries measurementSeries;
      for (const auto& measurement : log.getData()) {
        // Create measurements converting the torque back to the raw value by multiplying it a second time by the direction.
        measurementSeries.push_back(Measurement(measurement.getState().getJointPositionTicks(), measurement.getState().getJointVelocity(),
                                                measurement.getState().getCurrent(), measurement.getState().getJointTorque(),
                                                measurement.getState().getTemperature()));
      }
      measurementSeriesVector_.push_back(measurementSeries);
      ANYDRIVE_INFO("Read " << measurementSeries.size() << " measurements.");
    }

    // Make sure the average measured joint torque is zero in order to make the fitting
    // work even if the gear/joint encoder offset calibration is off.
    double averageJointTorque = 0.0;
    for (const auto& measurementSeries : measurementSeriesVector_) {
      for (const auto& measurement : measurementSeries) {
        averageJointTorque += measurement.jointTorque_ / measurementSeries.size();
      }
    }
    averageJointTorque /= measurementSeriesVector_.size();
    ANYDRIVE_INFO("Friction calibration: Measured average joint torque: " << averageJointTorque);
    for (auto& measurementSeries : measurementSeriesVector_) {
      for (auto& measurement : measurementSeries) {
        measurement.jointTorque_ -= averageJointTorque;
      }
    }
  }

  return true;
}

bool FrictionEstimation::postProcessData() {
  ANYDRIVE_INFO("Friction calibration: Fitting function.");
  bool ret;
  params_.setZero();
  ret = fitFunction(params_);
  if (writeToFiles_) {
    toFiles(params_);
  }
  return ret;
}

bool FrictionEstimation::fitFunction(Vector& params) const {
  Vector params0;
  getInitializationParams(params0);
  ANYDRIVE_INFO("Friction calibration: Initialization parameters:");
  ANYDRIVE_INFO(paramsToString(params0));

  Optimizer optimizer;
  double error = 0;
  bool success = optimizer.optimize(error, params, params0, std::bind(&FrictionEstimation::getError, this, std::placeholders::_1), true);

  ANYDRIVE_INFO("Friction calibration: Optimized parameters before check:");
  ANYDRIVE_INFO(paramsToString(params));

  success &= makeParamsUnique(params);

  ANYDRIVE_INFO("Friction calibration: Optimized parameters:");
  ANYDRIVE_INFO(paramsToString(params));
  ANYDRIVE_INFO("Friction calibration: Optimization residual error: " << error);

  return success;
}

void FrictionEstimation::getInitializationParams(Vector& params0) const {
  params0[0] = 0.0;
  params0[1] = 0.0;
  params0[2] = 0.0;
}

bool FrictionEstimation::makeParamsUnique(Vector& params) {
  bool success = true;
  // Check if the parameters are valid. If not, set all of them to zero.
  if (params[0] < 0.0) {
    ANYDRIVE_ERROR("The measured break away joint torque is negative.");
    params.setZero();
    success = false;
  }
  if (params[1] < 0.0) {
    ANYDRIVE_ERROR("The measured viscous friction coefficient for negative joint velocities is negative.");
    params.setZero();
    success = false;
  }
  if (params[2] < 0.0) {
    ANYDRIVE_ERROR("The measured viscous friction coefficient for positive joint velocities is negative.");
    params.setZero();
    success = false;
  }
  return success;
}

std::string FrictionEstimation::paramsToString(const Vector& params) {
  std::stringstream ss;
  ss << params[0] << "," << params[1] << "," << params[2];
  return ss.str();
}

double FrictionEstimation::smoothSign(const double x, const double xBand) {
  if (x < -xBand) {
    return -1.0;
  } else if (x > xBand) {
    return 1.0;
  } else {
    const float div = (x + xBand) / (2.0 * xBand);
    return -1.0 + (2.0 * div * div) * (2.0 - (x / xBand));
  }
}

bool FrictionEstimation::jointVelocityIsInsideBand(const double jointVelocity) const {
  const double jointVelocityRpm = jointVelocity / 2.0 / M_PI * 60.0;
  return std::abs(jointVelocityRpm) <= breakAwayTorqueBandRpm_;
}

double FrictionEstimation::getFrictionEstimation(const Vector& params, const double jointVelocity) const {
  const double jointVelocityRpm = jointVelocity / 2.0 / M_PI * 60.0;
  if (jointVelocityRpm < 0.0) {
    return params[0] * smoothSign(jointVelocityRpm, breakAwayTorqueBandRpm_) + params[1] * jointVelocityRpm;
  } else {
    return params[0] * smoothSign(jointVelocityRpm, breakAwayTorqueBandRpm_) + params[2] * jointVelocityRpm;
  }
}

double FrictionEstimation::getError(const Vector& params) const {
  double error = 0.0;
  for (const auto& measurementSeries : measurementSeriesVector_) {
    for (const auto& measurement : measurementSeries) {
      // Set errors to 0.0 if the joint velocity is within [-breakAwayTorqueBandRpm_, breakAwayTorqueBandRpm_].
      if (!jointVelocityIsInsideBand(measurement.jointVelocity_)) {
        const double frictionEstimation = getFrictionEstimation(params, measurement.jointVelocity_);
        error += std::pow(static_cast<double>(frictionEstimation - measurement.jointTorque_), 2);
      }
    }
  }
  return error;
}

std::string FrictionEstimation::getFilenameTrunk() const {
  return "friction_estimation_" + std::to_string(duration_);
}

void FrictionEstimation::toFiles(const Vector& params) const {
  for (unsigned int i = 0; i < motorVelocities_.size(); i++) {
    ANYDRIVE_INFO("Writing " << names_[i] << " measurement series to file.");
    const std::string filePath = filesFolder_ + "/" + filenameTrunk_ + "_" + names_[i] + ".txt";
    std::ofstream file;
    file.open(filePath);
    for (const auto& measurement : measurementSeriesVector_[i]) {
      file << measurement.jointPositionTicks_ << ",";
      file << measurement.jointVelocity_ << ",";
      file << measurement.current_ << ",";
      file << measurement.jointTorque_ << ",";
      file << getFrictionEstimation(params, measurement.jointVelocity_) << ",";
      file << measurement.temperature_;
      file << std::endl;
    }
    file.close();
    ANYDRIVE_INFO("Wrote " << measurementSeriesVector_[i].size() << " measurements to file.");
  }
}

void FrictionEstimation::fromFiles() {
  measurementSeriesVector_.clear();

  for (unsigned int i = 0; i < motorVelocities_.size(); i++) {
    ANYDRIVE_INFO("Reading " << names_[i] << " measurement series from file.");
    const std::string filePath = filesFolder_ + "/friction_estimation/" + filenameTrunk_ + "_" + names_[i] + ".txt";
    std::ifstream file;
    file.open(filePath);
    if (!file.is_open()) {
      ANYDRIVE_ERROR("File '" << filePath << "' could not be opened.");
      continue;
    }

    MeasurementSeries measurementSeries;
    int32_t jointPositionRawTicks = 0;
    double jointVelocity = 0.0;
    double current = 0.0;
    double jointTorque = 0.0;
    double dummy = 0.0;
    double temperature = 0.0;
    while (file >> jointPositionRawTicks >> jointVelocity >> current >> jointTorque >> dummy >> temperature) {
      measurementSeries.push_back(Measurement(jointPositionRawTicks, jointVelocity, current, jointTorque, temperature));
    }
    measurementSeriesVector_.push_back(measurementSeries);
    file.close();
    ANYDRIVE_INFO("Read " << measurementSeries.size() << " measurements from file.");
  }
}

}  // namespace routine
}  // namespace calibration
}  // namespace anydrive
