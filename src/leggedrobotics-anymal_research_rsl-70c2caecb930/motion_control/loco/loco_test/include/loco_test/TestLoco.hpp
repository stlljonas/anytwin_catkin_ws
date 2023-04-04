/*!
* @file    TestLoco.hpp
* @author  Gabriel Hottiger
* @date    Jan, 2018
*/

#pragma once

// gtest
#include <gtest/gtest.h>

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/common/arms/Arms.hpp"
#include "loco/common/legs/Legs.hpp"
#include "loco/common/torso/TorsoBase.hpp"

namespace loco_test {

class TestLoco : public ::testing::Test {
 public:
  explicit TestLoco(const double dt) : dt_(dt), wholeBody_(nullptr), torso_(nullptr), legs_(nullptr), arms_(nullptr) {}
  virtual ~TestLoco() = default;

  virtual void setupTest() = 0;
  virtual void setRandom() = 0;

 protected:

  void initialize(loco::WholeBody* wholeBody) {
    // Set ptrs
    wholeBody_ = wholeBody;
    torso_ = wholeBody->getTorsoPtr();
    legs_ = wholeBody->getLegsPtr();
    arms_ = wholeBody->getArmsPtr();

    // Initialize
    torso_->initialize(dt_);
    for (auto leg : *legs_) {
      leg->initialize(dt_);
    }
    for (auto arm : *arms_) {
      arm->initialize(dt_);
    }
    wholeBody_->initialize(dt_);
  }

  void advance() {
    torso_->advance(dt_);
    for (auto leg : *legs_) {
      leg->advance(dt_);
    }
    for (auto arm : *arms_) {
      arm->advance(dt_);
    }
    wholeBody_->advance(dt_);
  };

 protected:
  const double dt_;
  loco::WholeBody* wholeBody_;
  loco::TorsoBase* torso_;
  loco::Legs* legs_;
  loco::Arms* arms_;
};
}
