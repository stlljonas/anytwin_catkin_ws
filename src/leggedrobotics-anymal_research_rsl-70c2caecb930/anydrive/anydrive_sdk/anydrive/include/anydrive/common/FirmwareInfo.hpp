#pragma once

#include <cstdint>
#include <iostream>
#include <string>

namespace anydrive {
namespace common {

struct FirmwareInfo {
  uint8_t infoVersion_ = 0;
  std::string version_ = "";
  std::string fwHash_ = "";
  std::string channelId_ = "";
  std::string channelTid_ = "";
  std::string serialNumber_ = "";
  std::string keyId_ = "";
};

std::ostream& operator<<(std::ostream& ostream, const FirmwareInfo& info);

}  // namespace common
}  // namespace anydrive
