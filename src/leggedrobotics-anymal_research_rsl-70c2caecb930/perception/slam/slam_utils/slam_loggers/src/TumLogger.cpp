/*
 * TumLogger.cpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */

#include "slam_loggers/TumLogger.hpp"

// C++ standard library
#include <fstream>

// message logger
#include <message_logger/message_logger.hpp>

namespace slam_loggers {

TumLogger::TumLogger(const FileSystemConfig& fileSystemConfig) : BaseLogger(fileSystemConfig) {}

bool TumLogger::initLogFile() {
  // Set up output directory;
  createOutputDirectory();

  // Set full path of log files.
  trackedPosesFilename_ = buildUpFilename("_poses");
  trackedPosesInWorldFrameFilename_ = buildUpFilename("_poses_in_world");
  groundTruthPosesFilename_ = buildUpFilename("_ground_truth_poses");

  // Delete current files to prevent corruption of data.
  std::remove(trackedPosesFilename_.c_str());
  std::remove(trackedPosesInWorldFrameFilename_.c_str());
  std::remove(groundTruthPosesFilename_.c_str());

  return true;
}

bool TumLogger::saveLogToFilesystem(const PoseLog& poseLog) {
  if (poseLog.robotInTrackedFramePoses_.empty()) {
    MELO_ERROR("No poses to append.");
    return false;
  }

  //! Header that is added to every log file, to specify the meaning of each column.
  const std::string poseLogFileHeader_ = "# timestamp x y z q_x q_y q_z q_w";

  if (!poseLog.robotInTrackedFramePoses_.empty()) {
    appendMessageToFile(poseLogFileHeader_, trackedPosesFilename_);
    const size_t numPosesInLog = poseLog.robotInTrackedFramePoses_.size();
    for (size_t i = 0; i < numPosesInLog; i++) {
      const double stamp = poseLog.timestamps_[i].toSec();
      const Eigen::Vector3d translation(poseLog.robotInTrackedFramePoses_[i].translation());
      Eigen::Quaterniond rotation(poseLog.robotInTrackedFramePoses_[i].linear());
      rotation.normalize();

      appendDataToFile(stamp, translation, rotation, trackedPosesFilename_);
    }
  }

  if (!poseLog.robotInWorldFramePoses_.empty()) {
    appendMessageToFile(poseLogFileHeader_, trackedPosesInWorldFrameFilename_);
    const size_t numPosesInLog = poseLog.robotInWorldFramePoses_.size();
    for (size_t i = 0; i < numPosesInLog; i++) {
      const double stamp = poseLog.timestamps_[i].toSec();
      const Eigen::Vector3d translation(poseLog.robotInWorldFramePoses_[i].translation());
      Eigen::Quaterniond rotation(poseLog.robotInWorldFramePoses_[i].linear());
      rotation.normalize();

      appendDataToFile(stamp, translation, rotation, trackedPosesInWorldFrameFilename_);
    }
  }

  if (!poseLog.groundTruthPoses_.empty()) {
    appendMessageToFile(poseLogFileHeader_, groundTruthPosesFilename_);
    const size_t numPosesInLog = poseLog.groundTruthPoses_.size();
    for (size_t i = 0; i < numPosesInLog; i++) {
      const double stamp = poseLog.timestamps_[i].toSec();
      const Eigen::Vector3d translation(poseLog.groundTruthPoses_[i].translation());
      Eigen::Quaterniond rotation(poseLog.groundTruthPoses_[i].linear());
      rotation.normalize();

      appendDataToFile(stamp, translation, rotation, groundTruthPosesFilename_);
    }
  }

  return true;
}

void TumLogger::appendDataToFile(const double& timestamp, const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation,
                                 const std::string& filename) {
  std::ofstream file;

  // Open file and set numerical precision to the max.
  file.open(filename, std::ios_base::app);
  file.precision(dbl::max_digits10);

  // Save data to file.
  file << timestamp << " ";
  file << translation.x() << " " << translation.y() << " " << translation.z() << " ";
  file << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.z() << std::endl;

  // Close file handle.
  file.close();
}

}  // namespace slam_loggers
