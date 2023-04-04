#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <anydrive/common/Macros.hpp>

namespace anydrive {

//! Simple templated log class, used for calibration.
template <typename DataT>
class Log {
 protected:
  //! Type of a time point.
  using TimePoint = std::chrono::system_clock::time_point;

  //! Name of the log for referencing.
  std::string name_;
  //! Time stamp of the log.
  TimePoint stamp_;
  //! Logged data.
  std::vector<DataT> data_;

 public:
  /*!
   * Constructor.
   * @param name Name of the log.
   */
  explicit Log(const std::string name) : name_(name), stamp_(std::chrono::system_clock::now()) {}

  /*!
   * Destructor.
   */
  virtual ~Log() = default;

  /*!
   * Get the name of the log.
   * @return Name of the log.
   */
  const std::string& getName() const { return name_; }

  /*!
   * Get the time stamp of the log.
   * @return Time stamp of the log.
   */
  const TimePoint& getStamp() const { return stamp_; }

  /*!
   * Get the logged data.
   * @return Logged data.
   */
  const std::vector<DataT>& getData() const { return data_; }

  /*!
   * Add data to log.
   * @param data Data to add.
   */
  void addData(const DataT& data) { data_.push_back(data); }
};

}  // namespace anydrive
