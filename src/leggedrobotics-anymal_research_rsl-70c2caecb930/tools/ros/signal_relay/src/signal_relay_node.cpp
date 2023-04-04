#include "signal_relay/SignalRelay.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "signal_relay");
  ros::NodeHandle nodeHandle("~");

  signal_relay::SignalRelay SignalRelay(nodeHandle);

  ros::spin();

  return EXIT_SUCCESS;
}
