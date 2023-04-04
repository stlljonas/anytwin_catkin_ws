#pragma once

#include <memory>
#include <mutex>

#include <anydrive/communication/CommunicationManagerBase.hpp>

#include <soem_interface/EthercatBusManagerBase.hpp>

#include <anydrive/setup/Setup.hpp>

namespace anydrive_ethercat {

class AnydriveManagerEthercat;

class AnydriveEthercatBusManager : public soem_interface::EthercatBusManagerBase, public anydrive::communication::CommunicationManagerBase {
 public:
  AnydriveEthercatBusManager() = default;
  ~AnydriveEthercatBusManager() override = default;

  bool loadSetup(const anydrive::setup::SetupPtr setup, anydrive::AnydriveManager* anydriveManager) override;

  bool startup() override;
  void updateRead() override;
  void updateWrite() override;
  void shutdown() override;
  bool isCommunicationOk() override;
  unsigned int getWorkingCounterTooLowCount() override;
};

using AnydriveEthercatBusManagerPtr = std::shared_ptr<AnydriveEthercatBusManager>;

}  // namespace anydrive_ethercat
