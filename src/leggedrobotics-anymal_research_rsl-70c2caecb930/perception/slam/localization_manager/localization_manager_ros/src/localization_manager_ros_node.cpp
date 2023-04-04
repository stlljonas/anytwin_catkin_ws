// localization manager ros
#include "localization_manager_ros/LocalizationManagerRos.hpp"

// any node
#include <any_node/any_node.hpp>

int main(int argc, char** argv) {
  any_node::Nodewrap<localization_manager_ros::LocalizationManagerRos> node(argc, argv, "localization_manager_ros", 2);
  return static_cast<int>(!node.execute());
}
