/*
 * KittiLogger.cpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */

#include "slam_loggers/KittiLogger.hpp"

// C++ standard library
#include <fstream>

// message logger
#include <message_logger/message_logger.hpp>

namespace slam_loggers {

KittiLogger::KittiLogger(const FileSystemConfig& fileSystemConfig) : BaseLogger(fileSystemConfig) {}

bool KittiLogger::initLogFile() {
  // Set up output directory;
  createOutputDirectory();

  // Set full path of log files.
  timestampsFilename_ = buildUpFilename("_timestamps");
  trackedPosesFilename_ = buildUpFilename("_poses");
  trackedPosesInWorldFrameFilename_ = buildUpFilename("_poses_in_world");
  groundTruthPosesFilename_ = buildUpFilename("_ground_truth_poses");

  // Delete current files to prevent corruption of data.
  std::remove(trackedPosesFilename_.c_str());
  std::remove(trackedPosesInWorldFrameFilename_.c_str());
  std::remove(groundTruthPosesFilename_.c_str());
  std::remove(timestampsFilename_.c_str());

  return true;
}

bool KittiLogger::saveLogToFilesystem(const PoseLog& poseLog) {
  if (poseLog.robotInTrackedFramePoses_.empty()) {
    MELO_ERROR("No poses to append.");
    return false;
  }

  //! Headers that are added to every log file, to specify the meaning of each column.
  const std::string timestampLogFileHeader_ = "# timestamp_0 .. timestamp_N";
  const std::string poseLogFileHeader_ = "# a b c d e f g h i j k l (homogeneous matrix rows)";

  if (!poseLog.robotInTrackedFramePoses_.empty()) {
    appendMessageToFile(poseLogFileHeader_, trackedPosesFilename_);
    appendMessageToFile(timestampLogFileHeader_, timestampsFilename_);
    const size_t numPosesInLog = poseLog.robotInTrackedFramePoses_.size();
    for (size_t i = 0; i < numPosesInLog; i++) {
      const double stamp = poseLog.timestamps_[i].toSec();
      const Eigen::Isometry3d& robotInTrackedFramePose = poseLog.robotInTrackedFramePoses_[i];

      appendPoseToFile(robotInTrackedFramePose.translation(), robotInTrackedFramePose.linear(), trackedPosesFilename_);
      appendTimestampToFile(stamp, timestampsFilename_);
    }
  }

  if (!poseLog.robotInWorldFramePoses_.empty()) {
    appendMessageToFile(poseLogFileHeader_, trackedPosesInWorldFrameFilename_);
    for (const Eigen::Isometry3d& pose : poseLog.robotInWorldFramePoses_) {
      appendPoseToFile(pose.translation(), pose.linear(), trackedPosesInWorldFrameFilename_);
    }
  }

  if (!poseLog.groundTruthPoses_.empty()) {
    appendMessageToFile(poseLogFileHeader_, groundTruthPosesFilename_);
    for (const Eigen::Isometry3d& pose : poseLog.groundTruthPoses_) {
      appendPoseToFile(pose.translation(), pose.linear(), groundTruthPosesFilename_);
    }
  }

  return true;
}

void KittiLogger::appendPoseToFile(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation, const std::string& filename) {
  std::ofstream file;

  // Open file and set numerical precision to the max.
  file.open(filename, std::ios_base::app);
  file.precision(dbl::max_digits10);

  // Save data to file.
  file << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " " << translation(0) << " ";
  file << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << " " << translation(1) << " ";
  file << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << " " << translation(2) << std::endl;

  // Close file handler.
  file.close();
}

void KittiLogger::appendTimestampToFile(const double& stamp, const std::string& filename) {
  std::ofstream file;

  // Open file and set numerical precision to the max.
  file.open(filename, std::ios_base::app);
  file.precision(dbl::max_digits10);

  // Save data to file.
  file << stamp << " " << std::endl;

  // Close file handler.
  file.close();
}

}  // namespace slam_loggers
