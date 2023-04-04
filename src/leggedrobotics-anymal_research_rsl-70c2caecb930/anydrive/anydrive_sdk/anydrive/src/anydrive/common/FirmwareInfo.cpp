#include <sstream>

#include "anydrive/common/FirmwareInfo.hpp"

namespace anydrive {
namespace common {

std::ostream& operator<<(std::ostream& ostream, const FirmwareInfo& info) {
  ostream << "Firmware info version:      " << static_cast<int>(info.infoVersion_) << std::endl;
  ostream << "Firmware version:           " << info.version_ << std::endl;
  ostream << "Firmware hash:              " << info.fwHash_ << std::endl;
  ostream << "Firmware channel id:        " << info.channelId_ << std::endl;
  ostream << "Firmware channel tid:       " << info.channelTid_ << std::endl;
  ostream << "Firmware serial number:     " << info.serialNumber_ << std::endl;
  ostream << "Firmware key id:            " << info.keyId_ << std::endl;
  return ostream;
}

}  // namespace common
}  // namespace anydrive
