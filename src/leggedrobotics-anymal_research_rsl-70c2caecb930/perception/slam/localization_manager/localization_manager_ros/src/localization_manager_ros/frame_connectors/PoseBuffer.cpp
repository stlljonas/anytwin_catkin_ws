// pose buffer
#include <localization_manager_ros/frame_connectors/PoseBuffer.hpp>

// logging
#include <message_logger/message_logger.hpp>

namespace localization_manager_ros {

void PoseBuffer::reset() {
  missCount_ = 0u;
  poses_.clear();
}

void PoseBuffer::add(const Time& time, const Pose& pose) {
  // Add new pose.
  poses_.insert(TimePosePair(time, pose));

  // Reduce size but leave at least one pose.
  while (isOversized()) {
    poses_.erase(poses_.begin());
  }
}

bool PoseBuffer::find(const Time& requestedTime, Pose& pose) {
  auto it = poses_.upper_bound(requestedTime);
  if (it == poses_.begin()) {
    // Buffer only contains poses after requestedTime.
    missCount_++;
    return false;
  }
  if (it == poses_.end()) {
    // Buffer does not contain any pose up-to-date with requestedTime.
    missCount_++;
    return false;
  }
  pose = (--it)->second;
  return true;
}

}  // namespace localization_manager_ros
