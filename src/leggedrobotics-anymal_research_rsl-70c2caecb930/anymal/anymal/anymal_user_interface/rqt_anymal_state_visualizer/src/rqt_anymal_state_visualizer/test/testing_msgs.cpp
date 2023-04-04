// std
#include <algorithm>
#include <random>

// ros
#include <ros/ros.h>

// msgs
#include <anymal_msgs/AnymalState.h>

bool isRosOk() {
  return ros::ok();
}

int main(int argc, char **argv) {
  // Set up ROS node.
  ros::init(argc, argv, "testing_msgs");
  ros::NodeHandle nodeHandle("~");

  // random
  // Seed with a real random value, if available
  std::random_device randomDevice;
  std::default_random_engine defaultRandomEngine(randomDevice());
  std::uniform_real_distribution<double> uniformRealDistribution(-10.0, 10.0);

  // Set up publishers.
  ros::Publisher publisher = nodeHandle.advertise<anymal_msgs::AnymalState>(
      "/state_estimator/anymal_state_throttle", 1);

  double sleepingLong = 1000.0;
  ros::Rate rate(sleepingLong);
  while (ros::ok()) {
    anymal_msgs::AnymalState anymalState;
    anymalState.header.stamp = ros::Time::now();
    // contacts
    for (int i = 0; i < 4; ++i) {
      anymal_msgs::Contact contact;
      contact.wrench.force.x = uniformRealDistribution(defaultRandomEngine);
      contact.wrench.force.y = uniformRealDistribution(defaultRandomEngine);
      contact.wrench.force.z = uniformRealDistribution(defaultRandomEngine);
      anymalState.contacts.push_back(contact);
    }
    publisher.publish(anymalState);
    rate.sleep();
  }

  return 0;
}
