/*
 * StateRosPublisherTest.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_anymal_common/AdapterAnymal.hpp"

// Free Gait
#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_ros/free_gait_ros.hpp>

// gtest
#include <gtest/gtest.h>

using namespace free_gait;

TEST(StateRosPublisher, DISABLED_Simple)
{
  ros::NodeHandle nodeHandle("~");
  AdapterRos adapterRos(nodeHandle, AdapterRos::AdapterType::Base);
  StateRosPublisher stateRosPublisher(nodeHandle, *adapterRos.getAdapterPtr());
  State state;

  ros::Duration duration(0.2);
  for (size_t i = 0; i < 100; ++i) {
    state.setRandom();
    stateRosPublisher.publish(state);
    duration.sleep();
  }
}
