/*!
 * @file    ImuInitializer.hpp
 * @author  Fabian Tresoldi
 * @date    July, 2019
 */

#pragma once

#include <Eigen/Dense>
#include <kindr/Core>
#include <message_logger/message_logger.hpp>

namespace anymal_state_estimator {

class ImuInitializer {
 public:

  ImuInitializer(unsigned int sampleSize,
                 unsigned int throwawaySampleSize,
                 unsigned int velocityCheckHistorySize,
                 double varLinAccWorldToImuInBaseFrameThreshold,
                 double varAngVelWorldToBaseInBaseFrameThreshold);
  
  bool getGyroBiasInBase(Eigen::Vector3d& gyroBiasInBaseFrame) const;
  bool overrideBaseToWorldRollAndPitch(kindr::RotationQuaternionD& orientationBaseToWorld) const;

  void addMeasurement(double measurementTime, const Eigen::Vector3d& measLinAccWorldToImuInBaseFrame, const Eigen::Vector3d& measAngVelWorldToBaseInBaseFrame);
  void reset();

  const Eigen::Vector3d& getMeanLinAccWorldToImuInBaseFrame() { return meanLinAccWorldToImuInBaseFrame_; }
  const Eigen::Vector3d& getVarLinAccWorldToImuInBaseFrame() { return varLinAccWorldToImuInBaseFrame_; }
  const double& getVarLinAccWorldToImuInBaseFrameThreshold() { return varLinAccWorldToImuInBaseFrameThreshold_; }
  const Eigen::Vector3d& getMeanAngAccWorldToImuInBaseFrame() { return meanAngVelWorldToBaseInBaseFrame_; }
  const Eigen::Vector3d& getVarAngAccWorldToImuInBaseFrame() { return varAngVelWorldToBaseInBaseFrame_; }
  const double& getVarAngVelWorldToBaseInBaseFrameThreshold() { return varAngVelWorldToBaseInBaseFrameThreshold_; }

 private:
  Eigen::Vector3d gyroBiasInBaseFrame_{Eigen::Vector3d::Zero()};
  kindr::RotationQuaternionD orientationBaseToWorld_{kindr::RotationQuaternionD()};

  // number of measurements to average
  const unsigned int sampleSize_{100u};
  // number of ignored samples to account for initial impacts
  const unsigned int throwawaySampleSize_{10u};

  // variance detection variables
  const unsigned int velocityCheckHistorySize_{200u};
  unsigned int velocityCheckCount_{0u};
  double varLinAccWorldToImuInBaseFrameThreshold_;
  double varAngVelWorldToBaseInBaseFrameThreshold_;
  Eigen::MatrixXd measLinAccWorldToImuInBaseFrameHistory_;
  Eigen::MatrixXd measAngVelWorldToBaseInBaseFrameHistory_;
  Eigen::Vector3d meanLinAccWorldToImuInBaseFrame_;
  Eigen::Vector3d varLinAccWorldToImuInBaseFrame_;
  Eigen::Vector3d meanAngVelWorldToBaseInBaseFrame_;
  Eigen::Vector3d varAngVelWorldToBaseInBaseFrame_;

  // state of the initializer
  unsigned int measurementCount_{0u};
  Eigen::Vector3d averageMeasAngVelWorldToBaseInBaseFrame_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d averageMeasLinAccWorldToImuInBaseFrame_{Eigen::Vector3d::Zero()};
  bool initializationFinished_{false};
  double now_{0.};

  const Eigen::Vector3d gravityInWorldNormalized_ = Eigen::Vector3d(0., 0., -1.);

  /*!
   * Checks for movement by keeping a history of accelerations and calcuatining their variance.
   * @param measLinAccWorldToImuInBaseFrame Linear acceleration of the base in base frame.
   * @param measAngVelWorldToBaseInBaseFrame Angular acceleration of the base in base frame.
   * @bool True no velocity detected, False is velocity is detected.
   */
  bool zeroVelocityCheck(const Eigen::Vector3d& measLinAccWorldToImuInBaseFrame, const Eigen::Vector3d& measAngVelWorldToBaseInBaseFrame);
  void addToHistory(const Eigen::Vector3d& sample, Eigen::MatrixXd& history);
  Eigen::Vector3d getHistoryMean(const Eigen::MatrixXd& history);
  Eigen::Vector3d getHistoryVariance(const Eigen::MatrixXd& history);
  static bool checkForThreshold(const Eigen::Vector3d& variance, const double threshold);

};

}  // namespace anymal_state_estimator
