/*******************************************************************************
 * This code gives an example of how to use the ANYdrive SDK Setup class.      *
 *                                                                             *
 * Implemented steps:                                                          *
 * - Instead of loading a setup from a file, create the setup programmatically *
 * - Instantiate the ANYdrive Manager in standalone mode                       *
 * - Load the ANYdrive setup                                                   *
 * - Start the ANYdrive Manager up                                             *
 * - Wait for 5 seconds                                                        *
 * - Shut the ANYdrive Manager down                                            *
 *******************************************************************************/

#include <anydrive/Exception.hpp>

#include "anydrive_ethercat/AnydriveManagerEthercat.hpp"
#include "anydrive_ethercat/setup/SetupEthercat.hpp"

// The main function.
int main(int argc, char** argv) {
  try {
    // Check if the number of arguments is correct.
    if (argc != 1) {
      ANYDRIVE_ERROR("Usage: " << argv[0]);  // NOLINT
      return 0;
    }

    // Set the process priority.
    anydrive::AnydriveManager::setProcessPriority(10);

    ANYDRIVE_INFO("ANYdrive EtherCAT setup example is starting ...");

    // ANYdrive Manager parameters, the setup path is read from the arguments.
    const bool standalone = true;
    const bool installSignalHandler = true;
    const double timeStep = 0.0025;

    // Create an EtherCAT setup.
    anydrive_ethercat::setup::SetupEthercatPtr setup(new anydrive_ethercat::setup::SetupEthercat());

    // The default setup contains a single ANYdrive at bus "eth0" and EtherCAT address 1.
    // The following line shows how to change its name to "anydrive1" and set its rotation direction:
    setup->anydrives_[0]->name_ = "anydrive1";
    setup->anydrives_[0]->configuration_.setDirection(1);

    // The ANYdrive's setup is stored as a pointer to anydrive::setup::Anydrive.
    // In order to get access to the full EtherCAT setup, we first need
    // to cast it to anydrive_ethercat::setup::AnydriveEthercat:
    anydrive_ethercat::setup::AnydriveEthercatPtr anydrive1 =
        std::static_pointer_cast<anydrive_ethercat::setup::AnydriveEthercat>(setup->anydrives_[0]);

    // Now we can access additional EtherCAT data, e.g. the EtherCAT PDO type:
    anydrive1->ethercatPdoTypeEnum_ = anydrive_ethercat::PdoTypeEnum::C;

    // Let us add another ANYdrive with EtherCAT address 2:
    anydrive_ethercat::setup::AnydriveEthercatPtr anydrive2(new anydrive_ethercat::setup::AnydriveEthercat());
    anydrive2->name_ = "anydrive2";
    anydrive2->ethercatAddress_ = 2;
    setup->anydrives_.push_back(anydrive2);

    // Create the ANYdrive Manager.
    anydrive_ethercat::AnydriveManagerEthercat anydriveManager(standalone, installSignalHandler, timeStep);

    // Load the ANYdrive setup from our setup class instance.
    if (!anydriveManager.loadSetup(setup)) {
      ANYDRIVE_ERROR("ANYdrive Manager could not load setup.");
      return 0;
    }

    // Start the ANYdrive Manager up.
    if (!anydriveManager.startup()) {
      ANYDRIVE_ERROR("ANYdrive Manager could not be started.");
      return 0;
    }

    // Run the ANYdrive Manager for 5 seconds.
    const double duration = 5.0;
    ANYDRIVE_INFO("Running for " << duration << " s.");
    anydrive::threadSleep(duration);

    // Shut the ANYdrive Manager down.
    anydriveManager.shutdown();

    ANYDRIVE_INFO("ANYdrive EtherCAT setup example has been shut down.");
  } catch (const anydrive::Exception& exception) {
    ANYDRIVE_ERROR("Caught an ANYdrive exception: " << exception.what());
  } catch (const std::runtime_error& exception) {
    ANYDRIVE_ERROR("Caught a standard runtime error: " << exception.what());
  } catch (...) {
    ANYDRIVE_ERROR("Caught an unknown exception.");
  }
  return 0;
}
