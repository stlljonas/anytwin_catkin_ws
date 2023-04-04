/*
 * PreviewTest.cpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_anymal_common/AdapterAnymal.hpp"

// Free Gait
#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_ros/free_gait_ros.hpp>

// Anymal model
#include <anymal_model/AnymalModel.hpp>
#include <anymal_model/AnymalState.hpp>

// gtest
#include <gtest/gtest.h>

using namespace free_gait;

using AD = anymal_description::AnymalDescription;

TEST(Preview, DISABLED_Simple)
{
  // Setup modules.
  ros::NodeHandle nodeHandle("~");
  AdapterRos adapterRos(nodeHandle, AdapterRos::AdapterType::Preview);
  AdapterBase& adapter(*adapterRos.getAdapterPtr());
  StepParameters parameters;
  StepCompleter completer(parameters, adapter);
  StepComputer computer;
  State state;
  Executor executor(completer, computer, adapter, state);
  StateRosPublisher stateRosPublisher(nodeHandle, adapter);

  // Define robot state.
  AdapterAnymal& anymalAdapter = dynamic_cast<AdapterAnymal&>(adapter);
  anymal_model::AnymalModel& model = anymalAdapter.getAnymalModel();
  anymal_model::AnymalState anymalState;
  anymalState.setRandom(); // TODO
  model.setState(anymalState, true, false, false);
  for (auto& contact : model.getContactContainer()) {
    contact->setState(AD::ContactStateEnum::CLOSED);
  }

  // Initialize executor.
  ASSERT_TRUE(executor.initialize());

  // Define motion.
  Step step;
  step.addBaseMotion(BaseAuto());
  executor.getQueue().add(step);

  // Execute.
  ros::Duration duration(0.1);
  for (size_t i = 0; i < 200; ++i) {
    executor.advance(duration.toSec());
    stateRosPublisher.publish(state);
    duration.sleep();
  }
}
