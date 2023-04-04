#pragma once

// std
#include <map>

// kindr
#include <kindr/Core>

// any measurements
#include <any_measurements/Time.hpp>

namespace localization_manager_ros {

/**
 * @brief      Helper class to keep track of the history of poses in odom based on their time stamp.
 */
class PoseBuffer {
 public:
  using Time = any_measurements::Time;
  using Pose = kindr::HomTransformQuatD;
  using TimePosePair = std::pair<Time, Pose>;

  /* Constructor and destructor */
  explicit PoseBuffer(double size) : size_(size) {}
  ~PoseBuffer() = default;

  /* Accessors */
  size_t getSize() const { return poses_.size(); }
  unsigned long int getMissCount() const { return missCount_; }
  bool isEmpty() const { return poses_.empty(); }

  /**
   * @brief Resets the buffer. Deletes all content and resets counters.
   *
   */
  void reset();

  /**
   * @brief Adds an element to the buffer.
   * @remark If the buffer has reached its size limit, the first element will be removed to keep the size bounded.
   *
   * @param time  Capture time of the element, used as a key. Must be unique, otherwise data will be overwritten.
   * @param pose  Pose that will be added to the buffer.
   */
  void add(const Time& time, const Pose& pose);

  /**
   * @brief Finds an element in the buffer.
   *
   * @param requestedTime
   * @param pose
   * @return true         If the element could be found, false otherwise.
   */
  bool find(const Time& requestedTime, Pose& pose);

 private:
  /**
   * @brief Checks if the buffer length is higher than the size limit.
   *
   * @return true   If the buffer is oversized, false otherwise.
   */
  bool isOversized() const {
    // true if buffer spans more time than size_ and is larger than 1
    return (poses_.size() > 1) && ((poses_.rbegin()->first - poses_.begin()->first).toSeconds() > size_);
  }

  unsigned long int missCount_{0};
  const double size_;
  std::map<Time, Pose> poses_;
};

}  // namespace localization_manager_ros
