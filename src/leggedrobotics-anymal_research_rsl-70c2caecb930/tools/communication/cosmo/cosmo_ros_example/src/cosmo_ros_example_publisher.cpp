//
// Created by pleemann on 25.05.17.
//

#ifndef COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL2
#endif

#include "cosmo_ros/cosmo_ros.hpp"

#include "cosmo_ros_example/TestMessage.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <ctime>

#include <ros/ros.h>

std::atomic<bool> g_running{true};

void signalCallback(int) {
  g_running = false;
}

int main(int argc, char** argv) {
  signal(SIGINT, signalCallback);

  std::srand(std::time(nullptr));  // use current time as seed for random generator
  ros::init(argc, argv, "talker" + std::to_string(std::rand()), ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // advertise with the static sized non-ros message struct. the toRos(..) and fromRos(..) functions (see TestMessage.hpp) are then used to
  // generate the ros message
  cosmo_ros::PublisherRosPtr<shared_memory::TestMessage, cosmo_ros_example::TestMessage, cosmo_ros_example::ConversionTraits> pub =
      cosmo_ros::advertiseShmRos<shared_memory::TestMessage, cosmo_ros_example::TestMessage, cosmo_ros_example::ConversionTraits>(
          nh, "test", "/test");
  shared_memory::TestMessage msg{};

  while (g_running) {
    ++msg.a_;
    MELO_INFO_STREAM("Publisher sends a: " << msg.a_);
    // send the message over shared memory and enqueue it to the outgoing ROS messages queue
    pub->publish(msg);

    // do the sending over ROS
    pub->sendRos();

    sleep(1);
  }

  return 0;
}
