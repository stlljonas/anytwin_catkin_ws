// C++ standard library
#include <memory>

// google test
#include <gtest/gtest.h>

// icp tools
#include "signal_relay/SignalRelay.hpp"

class SignalRelaySetupTest : public ::testing::Test {
 protected:
  using SignalRelay = signal_relay::SignalRelay;

  std::unique_ptr<SignalRelay> signalRelay_;
  std::unique_ptr<ros::NodeHandle> nodeHandle_;

  void SetUp() override {
    nodeHandle_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));
    signalRelay_ = std::unique_ptr<SignalRelay>(new SignalRelay(*nodeHandle_));
  }

  void TearDown() override {
    signalRelay_.release();
    nodeHandle_.release();
  }
};

TEST_F(SignalRelaySetupTest, ReadParameters) {  // NOLINT
  ASSERT_FALSE(signalRelay_->readParameters());
}
