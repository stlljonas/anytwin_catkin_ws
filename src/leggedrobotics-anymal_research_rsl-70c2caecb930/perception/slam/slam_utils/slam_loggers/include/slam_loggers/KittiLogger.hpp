/*
 * KittiLogger.hpp
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
 * @brief Logs odometry/SLAM data (localization poses, optional ground truth) in KITTI Odometry Benchmark format. Timestamps
 * can also be saved.
 *
 *  Format:
 *    KITTI log files contain only poses, encoded as SE3 transformation matrices. Practically, for a given
 *    transformation matrix:
 *              a b c d
 *              e f g h
 *              i j k l
 *              0 0 0 1
 *    The following line would be written on the log file:
 *       a b c d e f g h i j k l
 *
 *    The timestamps of the poses are also saved to another log file, as follows:
 *       timestamp_1
 *       timestamp_2
 *       ...
 *       timestamp_n
 *
 */
class KittiLogger : public BaseLogger {
 public:
  SMART_POINTERS_TYPEDEF(KittiLogger);
  using dbl = std::numeric_limits<double>;

  /**
   * @copydoc slam_loggers::BaseLogger::BaseLogger()
   */
  explicit KittiLogger(const FileSystemConfig& fileSystemConfig);

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
   * @brief Appends a pose to log file.
   *
   * @param translation Pose translation vector.
   * @param rotation Pose rotation matrix.
   * @param filename The log filename.
   */
  void appendPoseToFile(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation, const std::string& filename);

  /**
   * @brief Appends a timestamp to log file.
   *
   * @param timestamp Timestamp, in seconds.
   * @param filename The log filename.
   */
  void appendTimestampToFile(const double& stamp, const std::string& filename);

 protected:
  std::string timestampsFilename_;
};

}  // namespace slam_loggers
