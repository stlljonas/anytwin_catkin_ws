/**
 * @file        JointLimitsTest.cpp
 * @authors     Prajish Sankar
 * @date        Jul 20, 2020
 * @affiliation ANYbotics
 */

// gtest
#include <gtest/gtest.h>

// loco anymal
#include "loco_anymal/testing/CommonFixture.hpp"

namespace loco_anymal {

//! @class Tests setting and getting valid joint position and velocity limits.
class JointLimitsTest : public CommonFixture {};

TEST_F(JointLimitsTest, testLimits) {
  using JointEnum = anymal_description::AnymalDescription::JointEnum;

  auto limbHAA = [](unsigned limbId) {
    switch (limbId) {
      case 0u:
        return JointEnum::LF_HAA;
      case 1u:
        return JointEnum::RF_HAA;
      case 2u:
        return JointEnum::LH_HAA;
      case 3u:
        return JointEnum::RH_HAA;
    }
  };

  auto limbHFE = [](unsigned limbId) {
    switch (limbId) {
      case 0u:
        return JointEnum::LF_HFE;
      case 1u:
        return JointEnum::RF_HFE;
      case 2u:
        return JointEnum::LH_HFE;
      case 3u:
        return JointEnum::RH_HFE;
    }
  };

  auto limbKFE = [](unsigned limbId) {
    switch (limbId) {
      case 0u:
        return JointEnum::LF_KFE;
      case 1u:
        return JointEnum::RF_KFE;
      case 2u:
        return JointEnum::LH_KFE;
      case 3u:
        return JointEnum::RH_KFE;
    }
  };

  for (unsigned limbId = 0u; limbId < wholeBody_->getLimbs().size(); ++limbId) {
    legs_->getLegPtrById(limbId)->initialize(dt_);

    const JointPositions& jointMinPositions = wholeBody_->getLimbs().get(limbId).getLimbStateMeasured().getJointMinPositions();
    model_->getLimitsAnymal()->getJointMinPosition(limbHAA(limbId));
    EXPECT_FALSE(isnan(jointMinPositions.x()));
    EXPECT_FALSE(isnan(jointMinPositions.y()));
    EXPECT_FALSE(isnan(jointMinPositions.z()));
    EXPECT_DOUBLE_EQ(jointMinPositions.x(), model_->getLimitsAnymal()->getJointMinPosition(limbHAA(limbId)));
    EXPECT_DOUBLE_EQ(jointMinPositions.y(), model_->getLimitsAnymal()->getJointMinPosition(limbHFE(limbId)));
    EXPECT_DOUBLE_EQ(jointMinPositions.z(), model_->getLimitsAnymal()->getJointMinPosition(limbKFE(limbId)));

    const JointPositions& jointMaxPositions = wholeBody_->getLimbs().get(limbId).getLimbStateMeasured().getJointMaxPositions();
    EXPECT_FALSE(isnan(jointMaxPositions.x()));
    EXPECT_FALSE(isnan(jointMaxPositions.y()));
    EXPECT_FALSE(isnan(jointMaxPositions.z()));
    EXPECT_DOUBLE_EQ(jointMaxPositions.x(), model_->getLimitsAnymal()->getJointMaxPosition(limbHAA(limbId)));
    EXPECT_DOUBLE_EQ(jointMaxPositions.y(), model_->getLimitsAnymal()->getJointMaxPosition(limbHFE(limbId)));
    EXPECT_DOUBLE_EQ(jointMaxPositions.z(), model_->getLimitsAnymal()->getJointMaxPosition(limbKFE(limbId)));

    const JointVelocities& jointMinVelocities = wholeBody_->getLimbs().get(limbId).getLimbStateMeasured().getJointMinVelocities();
    EXPECT_FALSE(isnan(jointMinVelocities.x()));
    EXPECT_FALSE(isnan(jointMinVelocities.y()));
    EXPECT_FALSE(isnan(jointMinVelocities.z()));
    EXPECT_DOUBLE_EQ(jointMinVelocities.x(), model_->getLimitsAnymal()->getJointMinVelocity(limbHAA(limbId)));
    EXPECT_DOUBLE_EQ(jointMinVelocities.y(), model_->getLimitsAnymal()->getJointMinVelocity(limbHFE(limbId)));
    EXPECT_DOUBLE_EQ(jointMinVelocities.z(), model_->getLimitsAnymal()->getJointMinVelocity(limbKFE(limbId)));

    const JointVelocities& jointMaxVelocities = wholeBody_->getLimbs().get(limbId).getLimbStateMeasured().getJointMaxVelocities();
    EXPECT_FALSE(isnan(jointMaxVelocities.x()));
    EXPECT_FALSE(isnan(jointMaxVelocities.y()));
    EXPECT_FALSE(isnan(jointMaxVelocities.z()));
    EXPECT_DOUBLE_EQ(jointMaxVelocities.x(), model_->getLimitsAnymal()->getJointMaxVelocity(limbHAA(limbId)));
    EXPECT_DOUBLE_EQ(jointMaxVelocities.y(), model_->getLimitsAnymal()->getJointMaxVelocity(limbHFE(limbId)));
    EXPECT_DOUBLE_EQ(jointMaxVelocities.z(), model_->getLimitsAnymal()->getJointMaxVelocity(limbKFE(limbId)));
  }
}

}  // namespace loco_anymal
