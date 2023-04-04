/*
 * FileSystemConfig.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

// C++ standard library
#include <string>

namespace slam_loggers {

struct FileSystemConfig {
  //! Directory in which logging files will be saved.
  std::string outputDirectory_ = "/tmp";

  //! Prefix for classifying logging files based on modules logged. (e.g. "slam")
  std::string moduleFilePrefix_ = "slam_module";

  //! Prefix for logging files, specifying session info or other options.
  std::string loggerFilePrefix_ = "session";

  //! File extension.
  std::string fileExtension_ = ".log";

  FileSystemConfig(std::string loggingPath, std::string moduleFilePrefix, std::string loggerFilePrefix, std::string fileExtension)
      : outputDirectory_(std::move(loggingPath)),
        moduleFilePrefix_(std::move(moduleFilePrefix)),
        loggerFilePrefix_(std::move(loggerFilePrefix)),
        fileExtension_(std::move(fileExtension)) {}
};

}  // namespace slam_loggers