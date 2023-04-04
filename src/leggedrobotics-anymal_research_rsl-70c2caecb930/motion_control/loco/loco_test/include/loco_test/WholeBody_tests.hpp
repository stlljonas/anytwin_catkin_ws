/*
 * WholeBody_tests.hpp
 *
 *  Created on: Jan 22, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco_test
#include "TestLoco.hpp"

// kindr
#include "kindr/common/gtest_eigen.hpp"

// gtest
#include <gtest/gtest.h>

namespace loco_test {

template <typename LocoFixtureType_>
class WholeBodyTest : public LocoFixtureType_ {
 public:
  //! Test
  void testGravityTerms() {
    const loco::TorsoBase &torso = this->wholeBody_->getTorso();

    loco::LinearAcceleration gravitationalAccelerationInBaseFrame =
        torso.getMeasuredState().getOrientationWorldToBase().rotate(torso.getProperties().getGravity());

    for (auto limb : this->wholeBody_->getLimbs()) {
      loco::JointTorques summedLinkGravityTorques = loco::JointTorques::Zero(limb->getNumDofLimb());
      for (auto link : limb->getLinks()) {
        summedLinkGravityTorques -=
            loco::JointTorques(link->getTranslationJacobianBaseToCoMInBaseFrame().transpose() *
                               loco::Force(link->getMass() * gravitationalAccelerationInBaseFrame).toImplementation());
      }
      loco::JointTorques limbGravityTorques = limb->getLimbStateMeasured().getGravityJointTorques();
      std::string msg = "Gravity Terms of " + limb->getName() + " are not equal to summed link gravity terms.";
      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(summedLinkGravityTorques, limbGravityTorques, 1.0, msg, 1e-6);
    }
  }
};

TYPED_TEST_CASE(WholeBodyTest, FIXTURE_TEST_TYPE);

TYPED_TEST(WholeBodyTest, TestGravityTermsInit) {
  this->setupTest();
  this->testGravityTerms();
}

TYPED_TEST(WholeBodyTest, TestGravityTermsAdvance) {
  this->setupTest();
  this->setRandom();
  this->testGravityTerms();
}

}
