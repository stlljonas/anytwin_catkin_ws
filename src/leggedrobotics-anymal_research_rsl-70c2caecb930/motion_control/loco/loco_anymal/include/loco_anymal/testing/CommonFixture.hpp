/**
 * @authors     Dario Bellicoso
 * @affiliation RSL
 * @brief       Common fixture for ANYmal loco tests
 */

#pragma once

// ros
#include <ros/package.h>

// gtest
#include <gtest/gtest.h>

// anymal_model
#include <anymal_model/AnymalModel.hpp>

// loco
#include <loco/gait_pattern/GaitPatternBase.hpp>

// loco anymal
#include "loco_anymal/loco_anymal.hpp"
#include "loco_anymal/typedefs.hpp"
#include "loco_anymal/common/WholeBodyAnymal.hpp"

namespace loco_anymal {

class CommonFixture : public ::testing::Test {
 public:
  //! Create models and containers
  void SetUp() override {
    // Create models
    model_.reset(new anymal_model::AnymalModel(dt_));
    modelDesired_.reset(new anymal_model::AnymalModel(dt_));

    // Load model description
    std::string path = ros::package::getPath("loco_anymal") + "/test/resources/anymal_c100_20200519.urdf";
    model_->initializeFromUrdf(path);
    modelDesired_->initializeFromUrdf(path);

    // Create containers
    legs_ = loco_anymal::make_unique_leg_group(*model_, *modelDesired_, false);
    torso_.reset(new loco_anymal::TorsoAnymal("torso", *model_));
    wholeBody_.reset(new loco_anymal::WholeBodyAnymal(*model_, *torso_, *legs_, true));
  }

 protected:
  //! Leg containers
  std::unique_ptr<loco_anymal::LegsAnymal> legs_;

  //! Torso container
  std::unique_ptr<loco_anymal::TorsoAnymal> torso_;

  //! Whole-body handle
  std::unique_ptr<loco_anymal::WholeBodyAnymal> wholeBody_;

  //! Model for measured state
  std::unique_ptr<anymal_model::AnymalModel> model_;

  //! Model for desired state
  std::unique_ptr<anymal_model::AnymalModel> modelDesired_;

  //! Time step (so that it's available in all test fixtures)
  double dt_ = 0.0025;
};

} // namespace loco_anymal
