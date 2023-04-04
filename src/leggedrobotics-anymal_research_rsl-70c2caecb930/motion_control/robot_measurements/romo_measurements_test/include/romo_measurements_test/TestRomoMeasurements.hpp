/*!
* @file    TestRomoMeasurements.hpp
* @author  Gabriel Hottiger
* @date    Jan, 2018
*/

#pragma once

// romo
#include "romo_rbdl/RobotModelRbdl.hpp"

// loco_test
#include "loco_test/TestLoco.hpp"

// gtest
#include <gtest/gtest.h>

// STL
#include <memory>

namespace romo_measurements_test {

template <typename RobotModel_>
class TestRomoMeasurements : public loco_test::TestLoco {
 public:
  using RobotModel = RobotModel_;
  using RobotState = typename RobotModel::RobotState;

  TestRomoMeasurements(std::string urdfPath, const double dt)
      : TestLoco(dt), urdfPath_(std::move(urdfPath)), model_(dt), desiredModel_(dt) {
  }

  virtual ~TestRomoMeasurements() = default;

  void setupTest() override {
    // TODO move initializeFromUrdf up to RobotModelRBDL then we could template the test on Description and RobotState
    ASSERT_TRUE(model_.initializeFromUrdf(urdfPath_));
    ASSERT_TRUE(desiredModel_.initializeFromUrdf(urdfPath_));

    // Set the desired state to zero.
    state_.setZero();
    desiredModel_.setState(state_, true, true);

    // Set the measured state to random.
    state_.setRandom();
    model_.setState(state_, true, true);
  }

  void setRandom() override {
    // Set the state to a random one.
    state_.setRandom();
    model_.setState(state_, true, true);
    // Advance the containers
    this->advance();
  }

 protected:
  const std::string urdfPath_;
  RobotModel model_;
  RobotModel desiredModel_;
  RobotState state_;
};
}
