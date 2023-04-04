// interaction marker
#include "interaction_marker/InteractionMarker.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "interaction_marker");
  ros::NodeHandle nodeHandle("~");
  interaction_marker::InteractionMarker interactionMarker(nodeHandle);
  ros::spin();
  return 0;
}
