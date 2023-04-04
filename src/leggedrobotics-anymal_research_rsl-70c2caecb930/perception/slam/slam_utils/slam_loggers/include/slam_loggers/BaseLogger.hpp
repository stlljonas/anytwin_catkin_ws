/*
 * BaseLogger.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

// slam common
#include <slam_common/macros.hpp>

// slam loggers
#include "slam_loggers/FileSystemConfig.hpp"
#include "slam_loggers/PoseLog.hpp"

namespace slam_loggers {

class BaseLogger {
 public:
  SMART_POINTERS_TYPEDEF(BaseLogger);

  /**
   * @brief Constructor.
   *
   * @param fileSystemConfig Configuration of logger output files.
   */
  explicit BaseLogger(FileSystemConfig fileSystemConfig);

  /**
   * @brief Default destructor.
   *
   */
  virtual ~BaseLogger() = default;

  /**
   * @brief Creates the output directory in the host filesystem, if it doesn't exist yet.
   *
   * @return true   If successful, false otherwise.
   */
  bool createOutputDirectory();

  /**
   * @brief Initialize logging file.
   * @remark Deletes files with similar names in the destionation folder, to prevent corruption of data.
   *
   * @return True if successful.
   */
  virtual bool initLogFile() = 0;

  /**
   * @brief Save poses logged to a file.
   *
   * @param poseLog A collection of poses logged and related data.
   * @return True if successful
   */
  virtual bool saveLogToFilesystem(const PoseLog& poseLog) = 0;

  /**
   * @brief Builds up the filename of a log file.
   *
   * @param typeSuffix    Type of logfile that will be saved (e.g. pose, sampled ground truth).
   * @return std::string  Filename.
   */
  virtual std::string buildUpFilename(const std::string& typeSuffix);

  /**
   * @brief Appends a message to a given file.
   *
   * @param message   Message to append.
   * @param filename  Path to the file.
   */
  void appendMessageToFile(const std::string& message, const std::string& filename);

 protected:
  //! Filesystem config (directory path, file prefix, extension, etc).
  FileSystemConfig fileSystemConfig_;

  //! Name of output files.
  std::string trackedPosesFilename_;
  std::string trackedPosesInWorldFrameFilename_;
  std::string groundTruthPosesFilename_;
};

}  // namespace slam_loggers
