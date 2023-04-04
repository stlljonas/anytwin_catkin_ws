/*******************************************************************************
 * This code gives a minimal example of how to use the ANYdrive SDK as         *
 * standalone application.                                                     *
 *                                                                             *
 * Implemented steps:                                                          *
 * - Instantiate the ANYdrive Manager in standalone mode                       *
 * - Load the ANYdrive setup from a file                                       *
 * - Start the ANYdrive Manager up                                             *
 * - Wait for 5 seconds                                                        *
 * - Shut the ANYdrive Manager down                                            *
 *******************************************************************************/

#include <anydrive/Exception.hpp>

#include "anydrive_ethercat/AnydriveManagerEthercat.hpp"

// The main function.
int main(int argc, char** argv) {
  try {
    // Check if the number of arguments is correct.
    if (argc < 1 || argc > 2) {
      ANYDRIVE_ERROR("Usage: " << argv[0] << " /path/to/your/setup.yaml=\"\"");  // NOLINT
      return 0;
    }

    // Set the process priority.
    anydrive::AnydriveManager::setProcessPriority(10);

    ANYDRIVE_INFO("ANYdrive EtherCAT minimal example is starting ...");

    // ANYdrive Manager parameters, the setup path is read from the arguments.
    const bool standalone = true;
    const bool installSignalHandler = true;
    const double timeStep = 0.0025;
    std::string setupFile;
    if (argc > 1) {
      setupFile = argv[1];  // NOLINT
    }

    // Create the ANYdrive Manager.
    anydrive_ethercat::AnydriveManagerEthercat anydriveManager(standalone, installSignalHandler, timeStep);

    // Load the ANYdrive setup from a file.
    if (!anydriveManager.loadSetup(setupFile)) {
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

    ANYDRIVE_INFO("ANYdrive EtherCAT minimal example has been shut down.");
  } catch (const anydrive::Exception& exception) {
    ANYDRIVE_ERROR("Caught an ANYdrive exception: " << exception.what());
  } catch (const std::runtime_error& exception) {
    ANYDRIVE_ERROR("Caught a standard runtime error: " << exception.what());
  } catch (...) {
    ANYDRIVE_ERROR("Caught an unknown exception.");
  }
  return 0;
}
