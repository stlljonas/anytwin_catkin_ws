/*
 * TumLogger.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

// C++ standard library
#include <limits>

// eigen
#include <Eigen/Dense>

// ros
#include <ros/ros.h>

// slam common
#include <slam_common/macros.hpp>

// slam loggers
#include "slam_loggers/BaseLogger.hpp"
#include "slam_loggers/FileSystemConfig.hpp"
#include "slam_loggers/PoseLog.hpp"

namespace slam_loggers {

/**
 * @brief Logs odometry/SLAM data (timestamps + localization poses or -optional- ground truth) in TUM format.
 *
 *  Format:
 *    TUM trajectory log files contain timestamps and poses. Practically, for a given timestamp (in seconds) and SE3
 *    pose (x,y,z, q_x, q_y, q_z, q_w), the following line would be written on the log file as a row with values:
 *       timestamp x y z q_x q_y q_z q_w
 *
 */
class TumLogger : public BaseLogger {
 public:
  SMART_POINTERS_TYPEDEF(TumLogger);
  using dbl = std::numeric_limits<double>;

  /**
   * @copydoc slam_loggers::BaseLogger::BaseLogger()
   */
  explicit TumLogger(const FileSystemConfig& fileSystemConfig);

  /**
   * @copydoc slam_loggers::BaseLogger::initLogFile()
   */
  bool initLogFile() override;

  /**
   * @copydoc slam_loggers::BaseLogger::saveLogToFilesystem()
   */
  bool saveLogToFilesystem(const PoseLog& poseLog) override;

 private:
  /**
   * @brief Appends new pose data to TUM trajectory file.
   *
   * @param timestamp Timestamp of pose.
   * @param translation Translation.
   * @param rotation Rotation, given as quaternion.
   * @param filename The trajectory log filename.
   */
  void appendDataToFile(const double& timestamp, const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation,
                        const std::string& filename);
};

}  // namespace slam_loggers
