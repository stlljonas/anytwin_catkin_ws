#pragma once

#include <map>

#include "anydrive/common/Log.hpp"

namespace anydrive {
namespace common {

//! Simple templated logger class, used for calibration.
template <typename DataT>
class Logger {
 protected:
  //! Type of a log.
  using LogT = Log<DataT>;
  //! Type of a map of logs.
  using LogTMap = std::map<std::string, LogT>;
  //! Type of a pointer to a log.
  using LogTPtr = LogT*;

  //! Map of logs.
  LogTMap logs_;
  //! Pointer to the active log. The null-pointer indicates no active log.
  LogTPtr activeLog_ = nullptr;

 public:
  /*!
   * Constructor.
   */
  Logger() = default;

  /*!
   * Destructor.
   */
  virtual ~Logger() = default;

  /*!
   * Check if logging is active.
   * @return True if logging is active.
   */
  bool logIsActive() const { return static_cast<bool>(activeLog_); }

  /*!
   * Get a log by name.
   * @param name Name of the log.
   * @return Log.
   */
  LogT getLog(const std::string& name) const {
    const auto it = logs_.find(name);
    if (it == logs_.end()) {
      ANYDRIVE_ERROR("Log with name '" << name << "' does not exist.");
      return LogT("");
    }
    return it->second;
  }

  /*!
   * Start a new log. Skips if a log with the given name already exists.
   * @param name Name of the new log.
   */
  void startLog(const std::string& name) {
    if (logIsActive()) {
      ANYDRIVE_ERROR("Log with name '" << activeLog_->getName() << "' is still active, cannot start a new one.");
      return;
    }

    const auto result = logs_.insert({name, LogT(name)});
    if (!result.second) {
      ANYDRIVE_ERROR("Log with name '" << name << "' exists already, will not overwrite it.");
      return;
    }

    activeLog_ = &result.first->second;
    ANYDRIVE_INFO("Started log with name '" << activeLog_->getName() << "'.");
  }

  /*!
   * Add data to the active log.
   * @param data Data.
   */
  void addDataToLog(const DataT& data) {
    if (!logIsActive()) {
      ANYDRIVE_ERROR("No log is active, cannot add data.");
      return;
    }
    activeLog_->addData(data);
  }

  /*!
   * Stop the active log.
   */
  void stopLog() {
    if (!logIsActive()) {
      ANYDRIVE_DEBUG("No log is active, cannot stop it.");
      return;
    }
    ANYDRIVE_INFO("Stopped log with name '" << activeLog_->getName() << "'.");
    activeLog_ = nullptr;
  }

  /*!
   * Clear all logs.
   */
  void clearLogs() {
    if (logIsActive()) {
      stopLog();
    }

    logs_.clear();
  }
};

}  // namespace common
}  // namespace anydrive
