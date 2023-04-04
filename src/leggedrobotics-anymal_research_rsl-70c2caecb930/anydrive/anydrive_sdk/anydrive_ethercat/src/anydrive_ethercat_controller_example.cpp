/*******************************************************************************
 * This code gives an example of how to use the ANYdrive SDK in sequence with  *
 * a simple controller.                                                        *
 *                                                                             *
 * Implemented steps:                                                          *
 * - Instantiate the ANYdrive Manager in non-standalone mode                   *
 * - Instantiate a worker for updating the ANYdrive Manager and the controller *
 * - Load the ANYdrive setup from a file                                       *
 * - Start the ANYdrive Manager up                                             *
 * - Wait for 5 seconds, while the worker is running                           *
 * - Shut the ANYdrive Manager down                                            *
 *******************************************************************************/

#include <anydrive/Exception.hpp>

#include "anydrive_ethercat/AnydriveManagerEthercat.hpp"

// Pointer to the ANYdrive Manager, needs to be accessed in main(..) and updateWorkerCb(..).
std::shared_ptr<anydrive_ethercat::AnydriveManagerEthercat> anydriveManager;

// The update worker calls this callback with a certain frequency.
// It updates the ANYdrive Manager and an example controller in sequence.
bool updateWorkerCb(const any_worker::WorkerEvent& /*event*/) {
  // Update the ANYdrive Manager manually, since "standalone" is false.
  if (!anydriveManager->update()) {
    return false;
  }

  // Set the goal states of all devices to ControlOp once.
  static bool setControlOp = false;
  if (!setControlOp) {
    anydriveManager->setGoalStatesEnum(anydrive::fsm::StateEnum::ControlOp);
    setControlOp = true;
  }

  // When all devices reached ControlOp, send commands.
  if (anydriveManager->allDevicesAreInTheState(anydrive::fsm::StateEnum::ControlOp)) {
    anydrive::Command command;
    command.setModeEnum(anydrive::mode::ModeEnum::MotorVelocity);
    command.setMotorVelocity(100.0);
    for (auto anydrive : anydriveManager->getAnydrives()) {
      if (anydrive->getActiveStateEnum() == anydrive::fsm::StateEnum::ControlOp) {
        // One can access the reading as follows:
        const double motorVelocity = anydrive->getReading().getState().getMotorVelocity();
        ANYDRIVE_INFO("ANYdrive '" << anydrive->getName() << "' has a motor velocity of '" << motorVelocity << "' rad/s.");

        // Stage the command for sending in the next update() call.
        anydrive->stageCommand(command);
      }
    }
  }

  return true;
}

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

    ANYDRIVE_INFO("ANYdrive EtherCAT controller example is starting ...");

    // ANYdrive Manager parameters, the setup path is read from the arguments.
    // Note: Standalone is false, since we want to call AnydriveManager::update()
    //       in our own thread in sequence with our controller.
    const bool standalone = false;
    const bool installSignalHandler = true;
    const double timeStep = 0.0025;
    std::string setupFile;
    if (argc > 1) {
      setupFile = argv[1];  // NOLINT
    }

    // Create the ANYdrive Manager.
    anydriveManager.reset(new anydrive_ethercat::AnydriveManagerEthercat(standalone, installSignalHandler, timeStep));

    // Create the update worker which calls "updateWorkerCb" at a frequency of "1.0/timeStep".
    // Note: It is not recommended ros::Rate::sleep() or ros::Duration::sleep(), since their
    //       sleep resolution is limited to 1ms.
    std::shared_ptr<any_worker::Worker> updateWorker;
    any_worker::WorkerOptions updateWorkerOptions;
    updateWorkerOptions.callback_ = std::bind(&updateWorkerCb, std::placeholders::_1);
    updateWorkerOptions.defaultPriority_ = 90;
    updateWorkerOptions.name_ = "ExampleUpdateWorker";
    updateWorkerOptions.timeStep_ = timeStep;
    updateWorkerOptions.enforceRate_ = true;
    updateWorker.reset(new any_worker::Worker(updateWorkerOptions));

    // Load the ANYdrive setup from a file.
    if (!anydriveManager->loadSetup(setupFile)) {
      ANYDRIVE_ERROR("ANYdrive Manager could not load setup.");
      return 0;
    }

    // Start the ANYdrive Manager up.
    if (!anydriveManager->startup()) {
      ANYDRIVE_ERROR("ANYdrive Manager could not be started.");
      return 0;
    }

    // Start the update worker.
    if (!updateWorker->start()) {
      throw anydrive::Exception("Update worker could not be started.");
    }

    // Run the ANYdrive Manager for 5 seconds.
    const double duration = 5.0;
    ANYDRIVE_INFO("Running for " << duration << " s.");
    anydrive::threadSleep(duration);

    //! Stop the update worker.
    updateWorker->stop();

    // Shut the ANYdrive Manager down.
    anydriveManager->shutdown();

    ANYDRIVE_INFO("ANYdrive EtherCAT controller example has been shut down.");
  } catch (const anydrive::Exception& exception) {
    ANYDRIVE_ERROR("Caught an ANYdrive exception: " << exception.what());
  } catch (const std::runtime_error& exception) {
    ANYDRIVE_ERROR("Caught a standard runtime error: " << exception.what());
  } catch (...) {
    ANYDRIVE_ERROR("Caught an unknown exception.");
  }
  return 0;
}
