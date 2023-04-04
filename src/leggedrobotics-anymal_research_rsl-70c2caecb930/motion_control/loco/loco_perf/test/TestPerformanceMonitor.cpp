/**
 * @authors     Stephane Caron
 * @affiliation ANYbotics
 * @brief       Tests for the locomotion performance monitor.
 */

// stl
#include <vector>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// gtest
#include <gtest/gtest.h>

// loco
#include <loco/testing/MockMissionControl.hpp>

// loco anymal
#include <loco_anymal/testing/CommonFixture.hpp>

// loco_perf
#include "loco_perf/PerformanceMonitor.hpp"

/*!
 * Test fixture class for the performance monitor.
 */
class TestPerformanceMonitor : public loco_anymal::CommonFixture {
 protected:
  //! Set up test cases.
  void SetUp() override {
    loco_anymal::CommonFixture::SetUp();

    missionController_ = std::make_unique<loco::MockMissionControl>();
    perfMonitor_ = std::make_unique<loco_perf::PerformanceMonitor>(*torso_, *missionController_);
    perfMonitor_->initialize(dt_);
  }

 protected:
  //! Mission controller
  std::unique_ptr<loco::MockMissionControl> missionController_ = nullptr;

  //! Performance monitor
  std::unique_ptr<loco_perf::PerformanceMonitor> perfMonitor_ = nullptr;
};

TEST_F(TestPerformanceMonitor, addVariablesToLog) {  // NOLINT
  ASSERT_TRUE(perfMonitor_->addVariablesToLog(""));
}

TEST_F(TestPerformanceMonitor, checkConfiguration) {  // NOLINT
  TiXmlHandle root = new TiXmlElement("Root");
  ASSERT_FALSE(perfMonitor_->loadParameters(root));

  TiXmlHandle performanceMonitor(root);
  if (tinyxml_tools::createChildElement(performanceMonitor, root, "PerformanceMonitor")) {
    TiXmlHandle twistTracking(performanceMonitor);
    if (tinyxml_tools::createChildElement(twistTracking, performanceMonitor, "TwistTracking")) {
      tinyxml_tools::writeParameter("", 42., twistTracking, "time_constant");
      ASSERT_TRUE(perfMonitor_->loadParameters(root));
    }
  }
}

TEST_F(TestPerformanceMonitor, isZeroOnInitialization) {  // NOLINT
  perfMonitor_->initialize(dt_);
  ASSERT_DOUBLE_EQ(perfMonitor_->getTwistTrackingError().getTranslationalVelocity().norm(), 0.);
}

TEST_F(TestPerformanceMonitor, readsTwists) {  // NOLINT
  Eigen::Vector3d firstTwistLinear = {1., 2., 3.};
  loco::Twist firstTwist(/* linear = */ firstTwistLinear, /* angular = */ Eigen::Vector3d{4., 5., 6.});
  loco::Twist secondTwist(/* linear = */ Eigen::Vector3d{6., 5., 4.}, /* angular = */ Eigen::Vector3d{3., 2., 1.});

  perfMonitor_->initialize(dt_);
  torso_->getMeasuredStatePtr()->inControlFrame().setAngularVelocityBaseInControlFrame(firstTwist.getRotationalVelocity());
  torso_->getMeasuredStatePtr()->inControlFrame().setLinearVelocityBaseInControlFrame(firstTwist.getTranslationalVelocity());
  perfMonitor_->advance(dt_);
  ASSERT_DOUBLE_EQ(perfMonitor_->getTwistTrackingError().getTranslationalVelocity().norm(), firstTwistLinear.norm());

  missionController_->setDesiredBaseTwistInControlFrame(firstTwist);
  perfMonitor_->advance(dt_);
  ASSERT_DOUBLE_EQ(perfMonitor_->getTwistTrackingError().getTranslationalVelocity().norm(), 0.);
  ASSERT_GE(perfMonitor_->getTwistTrackingErrorAverage().getTranslationalVelocity().norm(), 0.);
  ASSERT_GE(perfMonitor_->getTwistTrackingErrorStdDev().getTranslationalVelocity().norm(), 0.);
}
