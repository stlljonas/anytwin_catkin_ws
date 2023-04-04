#include <soem_interface_examples/ExampleSlave.hpp>
#include <soem_interface/EthercatBusBase.hpp>

// This shows a minimal example on how to use the soem_interface library. 
// Keep in mind that this is non-working example code, with only minimal error handling

int main(int argc, char** argv) {
  const std::string busName = "eth1";
  const std::string slaveName = "ExampleSlave";
  const uint32_t slaveAddress = 0;

  std::unique_ptr<soem_interface::EthercatBusBase> bus = std::make_unique<soem_interface::EthercatBusBase> (
    busName);

  std::shared_ptr<soem_interface_examples::ExampleSlave> slave = std::make_shared<soem_interface_examples::ExampleSlave> (
    slaveName, bus.get(), slaveAddress);

  bus->addSlave(slave);
  bus->startup();
  bus->setState(EC_STATE_OPERATIONAL);

  if(!bus->waitForState(EC_STATE_OPERATIONAL, slaveAddress)) {
    // Something is wrong
    return 1;
  }

  while(true) {
    bus->updateRead();
    bus->updateWrite();
  }

  bus->shutdown();
  return 0;
}