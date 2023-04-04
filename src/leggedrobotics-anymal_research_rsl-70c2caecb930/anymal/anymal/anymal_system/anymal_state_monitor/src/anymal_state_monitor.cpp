#include <anymal_state_monitor/AnymalStateMonitor.h>

using namespace anymal_state_monitor;

int main(int argc, char **argv) {
  ros::init(argc, argv, "anymal_state_monitor");
  ros::NodeHandle nh("~");

  AnymalStateMonitor anymalStateMonitor(nh);

  ros::spin();
  return 0;
}