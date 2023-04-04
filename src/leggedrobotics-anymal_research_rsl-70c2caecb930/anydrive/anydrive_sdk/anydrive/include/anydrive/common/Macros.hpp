#pragma once

#include <cassert>
#include <deque>
#include <iostream>
#include <mutex>

#include <message_logger/message_logger.hpp>

namespace anydrive {
namespace common {

class MacroHelpers {
 public:
  static inline std::string extractFunctionName(const std::string& prettyFunction) {
    std::string functionName = prettyFunction;

    // remove function arguments
    const size_t argsBegin = functionName.find_first_of('(');
    functionName = functionName.substr(0, argsBegin);

    // remove template parameters
    while (functionName.find_last_of('<') != std::string::npos) {
      const size_t tempBegin = functionName.find_last_of('<');
      const size_t tempLenth = functionName.substr(tempBegin, std::string::npos).find_first_of('>') + 1;
      functionName = functionName.substr(0, tempBegin) + functionName.substr(tempBegin + tempLenth, std::string::npos);
    }

    // remove return values
    const size_t classBegin = functionName.find_last_of(' ') + 1;
    return functionName.substr(classBegin, std::string::npos);
  }
};

class MessageLog {
 public:
  using Log = std::deque<std::pair<message_logger::log::levels::Level, std::string>>;

 protected:
  static constexpr size_t maxLogSize_ = 20;

  static std::mutex logMutex_;
  static Log log_;

 public:
  static void insertMessage(message_logger::log::levels::Level level, const std::string& message);
  static Log getLog();
  static void clearLog();
  static Log getAndClearLog();
};

}  // namespace common
}  // namespace anydrive

#ifndef NDEBUG
#define _ANYDRIVE_ADD_FUNCTION_NAME(message) \
  "[" << anydrive::common::MacroHelpers::extractFunctionName(__PRETTY_FUNCTION__) << "] " << message
#else
#define _ANYDRIVE_ADD_FUNCTION_NAME(message) message
#endif
#define _ANYDRIVE_ADD_NAME(message) "[" << getName() << "] " << message
#define _ANYDRIVE_TO_STRING(message) \
  std::stringstream stream;          \
  stream << message;                 \
  const std::string messageString = stream.str();

// ANYdrive output.
#define ANYDRIVE_DEBUG(message) \
  { MELO_DEBUG_STREAM(_ANYDRIVE_ADD_FUNCTION_NAME(message)); }
#define ANYDRIVE_INFO(message) \
  { MELO_INFO_STREAM(_ANYDRIVE_ADD_FUNCTION_NAME(message)); }
#define ANYDRIVE_WARN(message) \
  { MELO_WARN_STREAM(_ANYDRIVE_ADD_FUNCTION_NAME(message)); }
#define ANYDRIVE_ERROR(message) \
  { MELO_ERROR_STREAM(_ANYDRIVE_ADD_FUNCTION_NAME(message)); }

// Throttled ANYdrive output.
#define ANYDRIVE_DEBUG_THROTTLE(rate, message) \
  { MELO_DEBUG_THROTTLE_STREAM(rate, _ANYDRIVE_ADD_FUNCTION_NAME(message)); }
#define ANYDRIVE_INFO_THROTTLE(rate, message) \
  { MELO_INFO_THROTTLE_STREAM(rate, _ANYDRIVE_ADD_FUNCTION_NAME(message)); }
#define ANYDRIVE_WARN_THROTTLE(rate, message) \
  { MELO_WARN_THROTTLE_STREAM(rate, _ANYDRIVE_ADD_FUNCTION_NAME(message)); }
#define ANYDRIVE_ERROR_THROTTLE(rate, message) \
  { MELO_ERROR_THROTTLE_STREAM(rate, _ANYDRIVE_ADD_FUNCTION_NAME(message)); }

// Logged ANYdrive output.
#define ANYDRIVE_LOGGED_DEBUG(message)                                                              \
  {                                                                                                 \
    _ANYDRIVE_TO_STRING(message);                                                                   \
    anydrive::common::MessageLog::insertMessage(message_logger::log::levels::Debug, messageString); \
    ANYDRIVE_DEBUG(message);                                                                        \
  }
#define ANYDRIVE_LOGGED_INFO(message)                                                              \
  {                                                                                                \
    _ANYDRIVE_TO_STRING(message);                                                                  \
    anydrive::common::MessageLog::insertMessage(message_logger::log::levels::Info, messageString); \
    ANYDRIVE_INFO(message);                                                                        \
  }
#define ANYDRIVE_LOGGED_WARN(message)                                                              \
  {                                                                                                \
    _ANYDRIVE_TO_STRING(message);                                                                  \
    anydrive::common::MessageLog::insertMessage(message_logger::log::levels::Warn, messageString); \
    ANYDRIVE_WARN(message);                                                                        \
  }
#define ANYDRIVE_LOGGED_ERROR(message)                                                              \
  {                                                                                                 \
    _ANYDRIVE_TO_STRING(message);                                                                   \
    anydrive::common::MessageLog::insertMessage(message_logger::log::levels::Error, messageString); \
    ANYDRIVE_ERROR(message);                                                                        \
  }

// Named ANYdrive output.
#define ANYDRIVE_NAMED_DEBUG(message) \
  { ANYDRIVE_DEBUG(_ANYDRIVE_ADD_NAME(message)); }
#define ANYDRIVE_NAMED_INFO(message) \
  { ANYDRIVE_INFO(_ANYDRIVE_ADD_NAME(message)); }
#define ANYDRIVE_NAMED_WARN(message) \
  { ANYDRIVE_WARN(_ANYDRIVE_ADD_NAME(message)); }
#define ANYDRIVE_NAMED_ERROR(message) \
  { ANYDRIVE_ERROR(_ANYDRIVE_ADD_NAME(message)); }

// Named throttled ANYdrive output.
#define ANYDRIVE_NAMED_DEBUG_THROTTLE(rate, message) \
  { ANYDRIVE_DEBUG_THROTTLE(rate, _ANYDRIVE_ADD_NAME(message)); }
#define ANYDRIVE_NAMED_INFO_THROTTLE(rate, message) \
  { ANYDRIVE_INFO_THROTTLE(rate, _ANYDRIVE_ADD_NAME(message)); }
#define ANYDRIVE_NAMED_WARN_THROTTLE(rate, message) \
  { ANYDRIVE_WARN_THROTTLE(rate, _ANYDRIVE_ADD_NAME(message)); }
#define ANYDRIVE_NAMED_ERROR_THROTTLE(rate, message) \
  { ANYDRIVE_ERROR_THROTTLE(rate, _ANYDRIVE_ADD_NAME(message)); }

// Logged named ANYdrive output.
#define ANYDRIVE_LOGGED_NAMED_DEBUG(message) \
  { ANYDRIVE_LOGGED_DEBUG(_ANYDRIVE_ADD_NAME(message)); }
#define ANYDRIVE_LOGGED_NAMED_INFO(message) \
  { ANYDRIVE_LOGGED_INFO(_ANYDRIVE_ADD_NAME(message)); }
#define ANYDRIVE_LOGGED_NAMED_WARN(message) \
  { ANYDRIVE_LOGGED_WARN(_ANYDRIVE_ADD_NAME(message)); }
#define ANYDRIVE_LOGGED_NAMED_ERROR(message) \
  { ANYDRIVE_LOGGED_ERROR(_ANYDRIVE_ADD_NAME(message)); }

/*!
 * ANYdrive assertion, checked in Debug builds.
 * @param condition    Condition which must be true.
 * @param errorMessage Error message which is printed in case the condition is false.
 */
#define ANYDRIVE_ASSERT(condition, errorMessage) \
  { assert((condition) && (errorMessage)); }
