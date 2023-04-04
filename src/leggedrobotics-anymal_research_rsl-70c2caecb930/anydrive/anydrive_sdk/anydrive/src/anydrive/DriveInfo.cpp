#include "anydrive/DriveInfo.hpp"

namespace anydrive {

DriveInfo::DriveInfo(std::string serialNumber, std::string model, std::string name, const uint16_t id,
                     const common::Version& bootloaderVersion, const common::Version& firmwareVersion)
    : serialNumber_(std::move(serialNumber)),
      model_(std::move(model)),
      name_(std::move(name)),
      id_(id),
      bootloaderVersion_(bootloaderVersion),
      firmwareVersion_(firmwareVersion) {}

std::string& DriveInfo::getSerialNumber() {
  return serialNumber_;
}

const std::string& DriveInfo::getSerialNumber() const {
  return serialNumber_;
}

std::string& DriveInfo::getModel() {
  return model_;
}

const std::string& DriveInfo::getModel() const {
  return model_;
}

std::string& DriveInfo::getName() {
  return name_;
}

const std::string& DriveInfo::getName() const {
  return name_;
}

uint16_t& DriveInfo::getId() {
  return id_;
}

uint16_t DriveInfo::getId() const {
  return id_;
}

common::Version& DriveInfo::getBootloaderVersion() {
  return bootloaderVersion_;
}

const common::Version& DriveInfo::getBootloaderVersion() const {
  return bootloaderVersion_;
}

common::Version& DriveInfo::getFirmwareVersion() {
  return firmwareVersion_;
}

const common::Version& DriveInfo::getFirmwareVersion() const {
  return firmwareVersion_;
}

std::ostream& operator<<(std::ostream& ostream, const DriveInfo& driveInfo) {
  ostream << "Serial number:      " << driveInfo.getSerialNumber() << std::endl;
  ostream << "Model:              " << driveInfo.getModel() << std::endl;
  ostream << "Name:               " << driveInfo.getName() << std::endl;
  ostream << "ID:                 " << driveInfo.getId() << std::endl;
  ostream << "Bootloader version: " << driveInfo.getBootloaderVersion() << std::endl;
  ostream << "Firmware version:   " << driveInfo.getFirmwareVersion();
  return ostream;
}

}  // namespace anydrive
