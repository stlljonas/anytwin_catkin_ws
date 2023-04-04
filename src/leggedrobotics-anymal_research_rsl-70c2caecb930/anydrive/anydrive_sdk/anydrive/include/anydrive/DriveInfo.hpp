#pragma once

#include <string>

#include "anydrive/common/Version.hpp"

namespace anydrive {

class DriveInfo {
 protected:
  std::string serialNumber_;

  std::string model_;
  std::string name_;
  uint16_t id_ = 0;

  common::Version bootloaderVersion_;
  common::Version firmwareVersion_;

 public:
  DriveInfo() = default;
  DriveInfo(std::string serialNumber, std::string model, std::string name, const uint16_t id, const common::Version& bootloaderVersion,
            const common::Version& firmwareVersion);
  virtual ~DriveInfo() = default;

  std::string& getSerialNumber();
  const std::string& getSerialNumber() const;

  std::string& getModel();
  const std::string& getModel() const;

  std::string& getName();
  const std::string& getName() const;

  uint16_t& getId();
  uint16_t getId() const;

  common::Version& getBootloaderVersion();
  const common::Version& getBootloaderVersion() const;

  common::Version& getFirmwareVersion();
  const common::Version& getFirmwareVersion() const;
};

std::ostream& operator<<(std::ostream& ostream, const DriveInfo& driveInfo);

}  // namespace anydrive
