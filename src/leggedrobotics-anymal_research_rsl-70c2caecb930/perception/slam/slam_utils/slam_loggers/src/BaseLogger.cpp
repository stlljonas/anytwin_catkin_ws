/*
 * BaseLogger.cpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */

#include "slam_loggers/BaseLogger.hpp"

// C++ standard library
#include <fstream>

// boost
#include <boost/filesystem.hpp>

// message logger
#include <message_logger/message_logger.hpp>

namespace slam_loggers {

BaseLogger::BaseLogger(FileSystemConfig fileSystemConfig) : fileSystemConfig_(std::move(fileSystemConfig)) {}

bool BaseLogger::createOutputDirectory() {
  // Check if the output folder exists.
  if (!boost::filesystem::is_directory(fileSystemConfig_.outputDirectory_)) {
    // If the folder doesn't exist, create it.
    if (!boost::filesystem::create_directories(fileSystemConfig_.outputDirectory_)) {
      MELO_ERROR("Output folder '%s' doesn't exist and could not be created", fileSystemConfig_.outputDirectory_.c_str());
      return false;
    }
  }

  return true;
}

std::string BaseLogger::buildUpFilename(const std::string& typeSuffix) {
  std::string filename;

  // Add prefixes.
  filename = fileSystemConfig_.outputDirectory_ + "/" + fileSystemConfig_.moduleFilePrefix_;

  // Add session prefix if it is not empty.
  if (!fileSystemConfig_.loggerFilePrefix_.empty()) {
    filename += "_" + fileSystemConfig_.loggerFilePrefix_;
  }

  // Add suffix and extension.
  filename += typeSuffix + "." + fileSystemConfig_.fileExtension_;

  return filename;
}

void BaseLogger::appendMessageToFile(const std::string& message, const std::string& filename) {
  std::ofstream file;

  // Open file and set numerical precision to the max.
  file.open(filename, std::ios_base::app);

  // Save data to file.
  file << message << std::endl;

  // Close file handle.
  file.close();
}

}  // namespace slam_loggers
