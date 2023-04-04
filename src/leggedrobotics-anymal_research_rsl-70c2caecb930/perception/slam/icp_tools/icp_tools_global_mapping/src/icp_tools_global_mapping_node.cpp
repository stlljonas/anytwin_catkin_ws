// icp tools
#include "icp_tools_global_mapping/SlamNode.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "icp_tools", ros::init_options::NoSigintHandler);
  ros::NodeHandle nodeHandle("~");

  icp_tools_global_mapping::SlamNode slamNode(nodeHandle);
  ros::spin();

  return EXIT_SUCCESS;
}
