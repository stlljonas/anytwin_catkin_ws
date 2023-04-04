#include "anydrive/calibration/routine/SafeJointVelocity.hpp"

namespace anydrive {
namespace calibration {
namespace routine {

SafeJointVelocity::SafeJointVelocity(const AnydrivePtr& anydrive, const double jointVelocityAbs, const uint64_t turns)
    : CalibrationBase(anydrive), jointVelocityAbs_(jointVelocityAbs), turns_(turns), filenameTrunk_(getFilenameTrunk()) {
  assert(n_ == multipliers_.size());
  assert(n_ == names_.size());
}

bool SafeJointVelocity::collectData() {
  if (stopRequested()) {
    return false;
  }

  bool readFromFiles = false;
  if (readFromFiles) {
    //    fromFiles(); // TODO(remo)
  } else {
    if (!anydrive_->setGoalStateEnum(fsm::StateEnum::ControlOp, true, 5.0, 10.0)) {
      ANYDRIVE_ERROR("Could not enter goal state '" << fsm::stateEnumToName(fsm::StateEnum::ControlOp) << "'.");
      return false;
    }

    const double duration = 2.0 * M_PI * turns_ / jointVelocityAbs_;

    for (i_ = 0; i_ < n_; i_++) {
      if (stopRequested()) {
        return false;
      }

      const double jointVelocity = multipliers_[i_] * jointVelocityAbs_;
      ANYDRIVE_INFO("Turning with joint v = " << jointVelocity << " rad/s for " << turns_ << " turns (" << names_[i_]
                                              << " measurement series).");
      stageCommandJointVelocity(jointVelocity);
      if (!preemptableSleep(3.0)) {
        return false;
      }
      anydrive_->startLog(names_[i_]);
      if (!preemptableSleep(duration)) {
        return false;
      }
      anydrive_->stopLog();
      if (!preemptableSleep(1.0)) {
        return false;
      }
      ANYDRIVE_INFO("Stopping.");
      stageCommandDisable();
      if (!preemptableSleep(1.0)) {
        return false;
      }

      ANYDRIVE_INFO("Reading logs.");
      const LogReading& log = anydrive_->getLog(names_[i_]);
      MeasurementSeries measurementSeries;
      for (const auto& measurement : log.getData()) {
        // Create measurements converting the torque back to the raw value by multiplying it a second time by the direction.
        measurementSeries.push_back(Measurement(measurement.getState().getJointPositionTicks(), measurement.getState().getJointTorque(),
                                                measurement.getState().getTemperature()));
      }
      measurementSeriesVector_.push_back(measurementSeries);
      ANYDRIVE_INFO("Read " << measurementSeries.size() << " measurements.");
    }

    anydrive_->setGoalStateEnum(fsm::StateEnum::Calibrate, true, 2.0, 0.1);
  }

  return true;
}

bool SafeJointVelocity::postProcessData() {
  ANYDRIVE_INFO("Fitting function.");
  for (i_ = 0; i_ < n_; i_++) {
    Vector params = Vector::Zero();
    params.setZero();
    fitFunction(params);
  }
  //  if (writeToFiles_)
  //    toFiles(params);
  return true;
}

void SafeJointVelocity::fitFunction(Vector& params) {
  Vector params0;
  getInitializationParams(params0);
  ANYDRIVE_INFO("Safe joint velocity: Initialization parameters:");
  ANYDRIVE_INFO(paramsToString(params0));

  Optimizer optimizer;
  double error = 0;
  optimizer.optimize(error, params, params0, std::bind(&SafeJointVelocity::getError, this, std::placeholders::_1), true);

  ANYDRIVE_INFO("Safe joint velocity: Optimized parameters before check:");
  ANYDRIVE_INFO(paramsToString(params));

  makeParamsUnique(params);

  ANYDRIVE_INFO("Safe joint velocity: Optimized parameters:");
  ANYDRIVE_INFO(paramsToString(params));
  ANYDRIVE_INFO("Safe joint velocity: Optimization residual error: " << error);
}

void SafeJointVelocity::getInitializationParams(Vector& params0) const {
  double averageOffset = 0;
  unsigned int numberOfMeasurements = 0;
  for (const auto& measurement : measurementSeriesVector_[i_]) {
    averageOffset += measurement.jointTorque_;
    numberOfMeasurements++;
  }
  averageOffset /= numberOfMeasurements;

  params0[0] = averageOffset;
  params0[1] = 0.0;
  params0[2] = 0.0;
}

void SafeJointVelocity::makeParamsUnique(Vector& params) {
  if (params[1] < 0) {
    params[1] *= -1.0;
    params[2] += M_PI;
  }
  params[2] = wrapTwoPI(params[2]);
}

std::string SafeJointVelocity::paramsToString(const Vector& params) {
  std::stringstream ss;
  ss << params[0] << "," << params[1] << "," << params[2];
  return ss.str();
}

double SafeJointVelocity::getGravityCompensation(const Vector& params, const double jointPositionTicks) const {
  return params[0] + params[1] * std::sin(static_cast<double>(omega_ * jointPositionTicks + params[2]));
}

double SafeJointVelocity::getError(const Vector& params) const {
  double error = 0;
  for (const auto& measurement : measurementSeriesVector_[i_]) {
    const double gravityCompensation = getGravityCompensation(params, measurement.jointPositionRawTicks_);
    error += std::pow(static_cast<double>(gravityCompensation - measurement.jointTorque_), 2);
  }
  return error;
}

std::string SafeJointVelocity::getFilenameTrunk() const {
  return "gravity_compensation_" + std::to_string(jointVelocityAbs_) + "_" + std::to_string(turns_);
}

void SafeJointVelocity::toFiles(const Vector& params) const {
  for (unsigned int i = 0; i < n_; i++) {
    ANYDRIVE_INFO("Writing " << names_[i] << " measurement series to file.");
    const std::string filePath = filesFolder_ + "/" + filenameTrunk_ + "_" + names_[i] + ".txt";
    std::ofstream file;
    file.open(filePath);
    for (const auto& measurement : measurementSeriesVector_[i]) {
      file << measurement.jointPositionRawTicks_ << ",";
      file << measurement.jointTorque_ << ",";
      file << getGravityCompensation(params, measurement.jointPositionRawTicks_) << ",";
      file << measurement.temperature_;
      file << std::endl;
    }
    file.close();
    ANYDRIVE_INFO("Wrote " << measurementSeriesVector_[i].size() << " measurements to file.");
  }
}

void SafeJointVelocity::fromFiles() {
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
    int32_t jointPositionRawTicks = 0;
    double jointTorque = 0;
    double dummy = 0;
    double temperature = 0;
    while (file >> jointPositionRawTicks >> jointTorque >> dummy >> temperature) {
      measurementSeries.push_back(Measurement(jointPositionRawTicks, jointTorque, temperature));
    }
    measurementSeriesVector_.push_back(measurementSeries);
    file.close();
    ANYDRIVE_INFO("Read " << measurementSeries.size() << " measurements from file.");
  }
}

}  // namespace routine
}  // namespace calibration
}  // namespace anydrive
