#include <ros/package.h>

#include <any_worker/Rate.hpp>

#include <anydrive/Exception.hpp>

#include "anydrive_ethercat_ros/AnydriveManagerEthercatRos.hpp"

int main(int argc, char** argv) {
  try {
    // Set the process priority.
    anydrive::AnydriveManager::setProcessPriority(10);

    ANYDRIVE_INFO("ANYdrive EtherCAT ROS node is starting ...");

    // Initialize ROS.
    ros::init(argc, argv, "anydrive_ros_node", ros::init_options::NoSigintHandler);

    // Create a ROS node handle.
    ros::NodeHandle nh("~");

    // Set the ANYdrive Manager parameters. Some are read from the ROS parameter server.
    const bool standalone = true;
    const bool installSignalHandler = true;
    // Read the time step (Same default value as in the launch file).
    const auto timeStep = param_io::param<double>(nh, "time_step", 0.0025);
    // Read the ROS prefix (Same default value as in the launch file).
    const std::string rosPrefix = param_io::param<std::string>(nh, "ros_prefix", "/anydrive");
    // Read the setup path (Same default value as in the launch file).
    const std::string setupFile =
        param_io::param<std::string>(nh, "setup_file", ros::package::getPath("anydrive") + "/example_setups/example1/setup.yaml");

    // Create the ANYdrive Manager.
    anydrive_ethercat_ros::AnydriveManagerEthercatRos anydriveManager(standalone, installSignalHandler, timeStep, nh, rosPrefix);

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

    // Run the ANYdrive Manager, as long as no shutdown is requested.
    any_worker::Rate rate("anydrive_ethercat_ros", timeStep);
    while (!anydriveManager.shutdownRequested()) {
      ros::spinOnce();
      rate.sleep();
    }

    // Shut the ANYdrive Manager down.
    anydriveManager.shutdown();

    ANYDRIVE_INFO("ANYdrive EtherCAT ROS node has been shut down.");
  } catch (const anydrive::Exception& exception) {
    ANYDRIVE_ERROR("Caught an ANYdrive exception: " << exception.what());
  } catch (const std::exception& exception) {
    ANYDRIVE_ERROR("Caught a standard exception: " << exception.what());
  } catch (...) {
    ANYDRIVE_ERROR("Caught an unknown exception.");
  }
  return 0;
}
