#pragma once

#include <functional>

#include "anydrive/ReadingExtended.hpp"

namespace anydrive {

// The first parameter of these callbacks is the name of the device.
using ReadingCb = std::function<void(const std::string&, const ReadingExtended&)>;
using ErrorCb = std::function<void(const std::string&)>;
using ErrorRecoveredCb = std::function<void(const std::string&)>;
using FatalCb = std::function<void(const std::string&)>;
using FatalRecoveredCb = std::function<void(const std::string&)>;
using DeviceDisconnectedCb = std::function<void(const std::string&)>;
using DeviceReconnectedCb = std::function<void(const std::string&)>;

// Anonymous callbacks do not include the name of the device.
using AnonymousReadingCb = std::function<void(const ReadingExtended&)>;
using AnonymousErrorCb = std::function<void(void)>;

}  // namespace anydrive
