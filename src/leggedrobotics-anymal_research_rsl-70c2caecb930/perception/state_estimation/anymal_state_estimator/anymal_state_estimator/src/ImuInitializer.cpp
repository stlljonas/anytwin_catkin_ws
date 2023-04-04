/*!
 * @file    ImuInitializer.cpp
 * @author  Fabian Tresoldi
 * @date    July, 2019
 */

#include <anymal_state_estimator/anymal_state_estimator_utils/ImuInitializer.hpp>


namespace anymal_state_estimator {

ImuInitializer::ImuInitializer(const unsigned int sampleSize,
                               const unsigned int throwawaySampleSize,
                               const unsigned int velocityCheckHistorySize,
                               double varLinAccWorldToImuInBaseFrameThreshold,
                               double varAngVelWorldToBaseInBaseFrameThreshold):
  sampleSize_(sampleSize),
  throwawaySampleSize_(throwawaySampleSize),
  velocityCheckHistorySize_(velocityCheckHistorySize),
  varLinAccWorldToImuInBaseFrameThreshold_(varLinAccWorldToImuInBaseFrameThreshold),
  varAngVelWorldToBaseInBaseFrameThreshold_(varAngVelWorldToBaseInBaseFrameThreshold) {}

void ImuInitializer::reset() {
  initializationFinished_ = false;
  measurementCount_ = 0u;
  now_ = 0.;
  averageMeasAngVelWorldToBaseInBaseFrame_ = Eigen::Vector3d::Zero();
  averageMeasLinAccWorldToImuInBaseFrame_ = Eigen::Vector3d::Zero();
  orientationBaseToWorld_ = kindr::RotationQuaternionD();
  gyroBiasInBaseFrame_ = Eigen::Vector3d::Zero();
  velocityCheckCount_ = 0u;
  measLinAccWorldToImuInBaseFrameHistory_ = Eigen::MatrixXd::Zero(3, velocityCheckHistorySize_);
  measAngVelWorldToBaseInBaseFrameHistory_ = Eigen::MatrixXd::Zero(3, velocityCheckHistorySize_);
}

void ImuInitializer::addMeasurement(double measurementTime, const Eigen::Vector3d& measLinAccWorldToImuInBaseFrame, const Eigen::Vector3d& measAngVelWorldToBaseInBaseFrame) {
  if (!zeroVelocityCheck(measLinAccWorldToImuInBaseFrame, measAngVelWorldToBaseInBaseFrame)) {
    measurementCount_ = 0u;
    averageMeasAngVelWorldToBaseInBaseFrame_ = Eigen::Vector3d::Zero();
    averageMeasLinAccWorldToImuInBaseFrame_ = Eigen::Vector3d::Zero();
  }

  if(measurementTime > now_){

    now_ = measurementTime;
    measurementCount_++;

    if (measurementCount_ > throwawaySampleSize_) {
      const unsigned int numUsableMeasurements = measurementCount_ - throwawaySampleSize_;
      if (numUsableMeasurements >= 1) {
        averageMeasAngVelWorldToBaseInBaseFrame_ =
            ((numUsableMeasurements - 1) * averageMeasAngVelWorldToBaseInBaseFrame_ + measAngVelWorldToBaseInBaseFrame) / (numUsableMeasurements);
        averageMeasLinAccWorldToImuInBaseFrame_ =
            ((numUsableMeasurements - 1) * averageMeasLinAccWorldToImuInBaseFrame_ + measLinAccWorldToImuInBaseFrame) / (numUsableMeasurements);
      } else {
        MELO_WARN_THROTTLE_STREAM(
            2., "[ImuInitializer] IMU parameters are not consistent. Did not add measurements!");
        measurementCount_--;
      }
    }

    // set output if enough measurements are in
    const double averageMeasLinAccWorldToImuInBaseFrameNorm = averageMeasLinAccWorldToImuInBaseFrame_.norm();
    if ((measurementCount_ > sampleSize_) && (averageMeasLinAccWorldToImuInBaseFrameNorm > 0.)) {

      // get gyro bias estimate
      gyroBiasInBaseFrame_ = averageMeasAngVelWorldToBaseInBaseFrame_;

      // get roll and pitch estimate
      Eigen::Vector3d gravityInBaseNormalized = -averageMeasLinAccWorldToImuInBaseFrame_ / averageMeasLinAccWorldToImuInBaseFrameNorm;
      kindr::RotationQuaternionPD orientationBaseToWorld;
      orientationBaseToWorld_.setFromVectors(gravityInBaseNormalized, gravityInWorldNormalized_);

      initializationFinished_ = true;
    }
  }
}

bool ImuInitializer::getGyroBiasInBase(Eigen::Vector3d& gyroBiasInBaseFrame) const {
  if(initializationFinished_){
    gyroBiasInBaseFrame = gyroBiasInBaseFrame_;
    return true;
  } else {
    return false;
  }
}

bool ImuInitializer::overrideBaseToWorldRollAndPitch(kindr::RotationQuaternionD& orientationBaseToWorld) const {
  if (initializationFinished_) {
    // get yaw of the given pose
    const double givenYaw = kindr::EulerAnglesZyxD(orientationBaseToWorld).setUnique().yaw();
    // get roll and pitch of the estimated orientation from gravity
    const auto orientationBaseToWorldYpr = kindr::EulerAnglesZyxD(orientationBaseToWorld_).setUnique();
    // set new orientation of the given pose using original yaw and estimated roll/pitch
    orientationBaseToWorld = kindr::RotationQuaternionD(kindr::EulerAnglesZyxD(givenYaw, orientationBaseToWorldYpr.pitch(), orientationBaseToWorldYpr.roll()));
    return true;
  } else {
    return false;
  }
}

bool ImuInitializer::zeroVelocityCheck(const Eigen::Vector3d& measLinAccWorldToImuInBaseFrame, const Eigen::Vector3d& measAngVelWorldToBaseInBaseFrame) {
  if (velocityCheckHistorySize_ <= 0) {
    // Ignore the zeroVelocityCheck if the history size given is 0 or negative.
    return true;
  }

  addToHistory(measLinAccWorldToImuInBaseFrame, measLinAccWorldToImuInBaseFrameHistory_);
  addToHistory(measAngVelWorldToBaseInBaseFrame, measAngVelWorldToBaseInBaseFrameHistory_);

  velocityCheckCount_++;
  if (velocityCheckCount_ < velocityCheckHistorySize_) { return false; }

  // Required for logging.
  meanLinAccWorldToImuInBaseFrame_ = getHistoryMean(measLinAccWorldToImuInBaseFrameHistory_); 
  varLinAccWorldToImuInBaseFrame_ = getHistoryVariance(measLinAccWorldToImuInBaseFrameHistory_);
  // Required for logging.
  meanAngVelWorldToBaseInBaseFrame_ = getHistoryMean(measAngVelWorldToBaseInBaseFrameHistory_);
  varAngVelWorldToBaseInBaseFrame_ = getHistoryVariance(measAngVelWorldToBaseInBaseFrameHistory_);

  return checkForThreshold(varLinAccWorldToImuInBaseFrame_, varLinAccWorldToImuInBaseFrameThreshold_)
         && checkForThreshold(varAngVelWorldToBaseInBaseFrame_, varAngVelWorldToBaseInBaseFrameThreshold_);
}

void ImuInitializer::addToHistory(const Eigen::Vector3d& sample, Eigen::MatrixXd& history) {
  // TODO(Val): Make this a ring buffer to save on moving operations.
  const size_t historySize = history.cols();
  history.leftCols(historySize - 1) = history.rightCols(historySize - 1);
  history.col(historySize - 1) = sample;
}

Eigen::Vector3d ImuInitializer::getHistoryMean(const Eigen::MatrixXd& history) {
  return history.rowwise().mean();
}

Eigen::Vector3d ImuInitializer::getHistoryVariance(const Eigen::MatrixXd& history) {
  return (history.colwise() - getHistoryMean(history)).rowwise().norm() / velocityCheckHistorySize_;
}

bool ImuInitializer::checkForThreshold(const Eigen::Vector3d& variance, const double threshold) {
  return variance.sum() < threshold * variance.size();
}


}