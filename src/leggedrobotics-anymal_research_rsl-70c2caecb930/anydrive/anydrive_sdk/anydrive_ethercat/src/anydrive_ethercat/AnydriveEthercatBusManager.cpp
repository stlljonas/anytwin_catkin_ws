// #include <anydrive/AnydriveManager.hpp>

#include "anydrive_ethercat/AnydriveEthercatBusManager.hpp"

#include "anydrive_ethercat/AnydriveEthercatSlave.hpp"
#include "anydrive_ethercat/AnydriveManagerEthercat.hpp"
#include "anydrive_ethercat/setup/AnydriveEthercat.hpp"
#include "anydrive_ethercat/setup/SetupEthercat.hpp"

namespace anydrive_ethercat {

bool AnydriveEthercatBusManager::loadSetup(const anydrive::setup::SetupPtr setup, anydrive::AnydriveManager* anydriveManager) {
  ANYDRIVE_ASSERT(std::dynamic_pointer_cast<setup::SetupEthercat>(setup),
                  "The setup pointer must contain an anydrive_ethercat::setup::SetupEthercat object.");

  // Clear the map containing all buses.
  buses_.clear();

  // Loop through all ANYdrives and create their buses.
  for (const auto& anydriveSetup : setup->anydrives_) {
    const setup::AnydriveEthercatPtr& anydriveEthercatSetup = std::dynamic_pointer_cast<setup::AnydriveEthercat>(anydriveSetup);
    ANYDRIVE_ASSERT(anydriveEthercatSetup, "The ANYdrive setup pointer must contain an anydrive_ethercat::setup::AnydriveEthercat object.");
    const std::string busName = anydriveEthercatSetup->ethercatBus_;
    if (busName.empty()) {
      ANYDRIVE_ERROR("ANYdrive '" << anydriveSetup->name_ << "': The name of the bus is empty.");
      return false;
    }
    soem_interface::EthercatBusBase* bus;
    const auto& it = buses_.find(busName);
    if (it == buses_.end()) {
      // Create a new bus.
      bus = new soem_interface::EthercatBusBase(busName);
      buses_.insert(std::make_pair(busName, std::unique_ptr<soem_interface::EthercatBusBase>(bus)));
    } else {
      // Take existing bus.
      bus = it->second.get();
    }

    const anydrive::AnydrivePtr& anydrive = anydriveManager->getAnydrive(anydriveEthercatSetup->name_);
    AnydriveEthercatSlavePtr slave(
        new AnydriveEthercatSlave(anydrive, bus, anydriveEthercatSetup->ethercatAddress_, anydriveEthercatSetup->ethercatPdoTypeEnum_));
    if (!bus->addSlave(slave)) {
      return false;
    }
    anydrive->setCommunicationInterface(slave);
  }

  return true;
}

bool AnydriveEthercatBusManager::startup() {
  return startupAllBuses();
}

void AnydriveEthercatBusManager::updateRead() {
  readAllBuses();
}

void AnydriveEthercatBusManager::updateWrite() {
  writeToAllBuses();
}

void AnydriveEthercatBusManager::shutdown() {
  shutdownAllBuses();
}

bool AnydriveEthercatBusManager::isCommunicationOk() {
  return allBusesAreOk();
}

unsigned int AnydriveEthercatBusManager::getWorkingCounterTooLowCount() {
  return getGlobalWorkingCounterTooLowCount();
}

}  // namespace anydrive_ethercat
