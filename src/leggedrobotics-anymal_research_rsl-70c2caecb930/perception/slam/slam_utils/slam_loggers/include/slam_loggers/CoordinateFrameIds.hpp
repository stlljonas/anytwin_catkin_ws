/*
 * CoordinateFrameIds.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

// C++ standard library
#include <string>

namespace slam_loggers {

// Struct for storing information about coordinate frames.
class CoordinateFrameIds {
 public:
  std::string& robotNameMutable() { return robotName_; }
  std::string& worldFrameIdMutable() { return worldFrameId_; }
  std::string& trackedFrameIdMutable() { return trackedFrameId_; }
  std::string& intermediateFrameIdMutable() { return intermediateFrameId_; }
  std::string& robotFrameIdMutable() { return robotFrameId_; }

  const std::string& robotName() const { return robotName_; }
  const std::string& worldFrameId() const { return worldFrameId_; }
  const std::string& trackedFrameId() const { return trackedFrameId_; }
  const std::string& intermediateFrameId() const { return intermediateFrameId_; }
  const std::string& robotFrameId() const { return robotFrameId_; }

 private:
  //! The name of the robot
  std::string robotName_ = "anymal";

  //! World frame id.
  std::string worldFrameId_ = "world";

  //! Frame in which the robot pose will be tracked.
  std::string trackedFrameId_ = "map";

  //! A child frame of the world frame. Can be a parent of the tracked and robot frames in some systems. (e.g. "odom")
  std::string intermediateFrameId_ = "intermediate";

  //! Robot frame id.
  std::string robotFrameId_ = "base";
};

}  // namespace slam_loggers