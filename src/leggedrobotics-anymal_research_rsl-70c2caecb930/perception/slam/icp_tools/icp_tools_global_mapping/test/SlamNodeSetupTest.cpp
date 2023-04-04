
// C++ standard library
#include <memory>

// google test
#include <gtest/gtest.h>

// icp tools
#include "icp_tools/SlamNode.hpp"

namespace icp_tools_global_mapping {

class SlamNodeSetupTest : public ::testing::Test {
 protected:
  std::unique_ptr<SlamNode> slamNode_;
  std::unique_ptr<ros::NodeHandle> nodeHandle_;

  void SetUp() override {
    nodeHandle_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));
    slamNode_ = std::unique_ptr<SlamNode>(new SlamNode(*nodeHandle_));
  }

  void disableAllFunctionality() {
    slamNode_->toggleLocalization(false);
    slamNode_->togglePublishPose(false);
    slamNode_->toggleMapping(false);
  }

  void enableAllFunctionality() {
    // Localization + Publish pose + Mapping = LMP
    slamNode_->toggleLocalization(true);
    slamNode_->togglePublishPose(true);
    slamNode_->toggleMapping(true);
  }
};

TEST_F(SlamNodeSetupTest, AllFunctionalityDisabled) {  // NOLINT
  ASSERT_FALSE(slamNode_->localizationIsEnabled());
  ASSERT_FALSE(slamNode_->publishPoseIsEnabled());
  ASSERT_FALSE(slamNode_->mappingIsEnabled());
}

TEST_F(SlamNodeSetupTest, EnableLocalization) {  // NOLINT
  // Publish pose disabling during full operation (LMP).
  {
    enableAllFunctionality();
    ASSERT_TRUE(slamNode_->toggleLocalization(false));

    ASSERT_FALSE(slamNode_->localizationIsEnabled());
    ASSERT_FALSE(slamNode_->publishPoseIsEnabled());
    ASSERT_FALSE(slamNode_->mappingIsEnabled());
  }

  // Localization enabling. Publish pose and Mapping disabled.
  {
    disableAllFunctionality();
    ASSERT_TRUE(slamNode_->toggleLocalization(true));

    ASSERT_TRUE(slamNode_->localizationIsEnabled());
    ASSERT_FALSE(slamNode_->publishPoseIsEnabled());
    ASSERT_FALSE(slamNode_->mappingIsEnabled());
  }
}

TEST_F(SlamNodeSetupTest, EnablePublishPose) {  // NOLINT
  // Publish pose disabling during full operation (LMP).
  {
    enableAllFunctionality();
    ASSERT_TRUE(slamNode_->togglePublishPose(false));

    ASSERT_TRUE(slamNode_->localizationIsEnabled());
    ASSERT_FALSE(slamNode_->publishPoseIsEnabled());
    ASSERT_TRUE(slamNode_->mappingIsEnabled());
  }

  // Publish pose enabling. Localization and Mapping disabled.
  {
    disableAllFunctionality();
    ASSERT_FALSE(slamNode_->togglePublishPose(true));

    ASSERT_FALSE(slamNode_->localizationIsEnabled());
    ASSERT_FALSE(slamNode_->publishPoseIsEnabled());
    ASSERT_FALSE(slamNode_->mappingIsEnabled());
  }

  // Localization and publish pose enabling
  {
    disableAllFunctionality();
    ASSERT_TRUE(slamNode_->toggleLocalization(true));
    ASSERT_TRUE(slamNode_->togglePublishPose(true));

    ASSERT_TRUE(slamNode_->localizationIsEnabled());
    ASSERT_TRUE(slamNode_->publishPoseIsEnabled());
    ASSERT_FALSE(slamNode_->mappingIsEnabled());
  }
}

TEST_F(SlamNodeSetupTest, EnableMapping) {  // NOLINT
  // Mapping disabling during full operation (LMP).
  {
    enableAllFunctionality();
    ASSERT_TRUE(slamNode_->toggleMapping(false));

    ASSERT_TRUE(slamNode_->localizationIsEnabled());
    ASSERT_TRUE(slamNode_->publishPoseIsEnabled());
    ASSERT_FALSE(slamNode_->mappingIsEnabled());
  }

  // Mapping enabling. Localization and publish pose disabled.
  {
    disableAllFunctionality();
    ASSERT_FALSE(slamNode_->toggleMapping(true));

    ASSERT_FALSE(slamNode_->localizationIsEnabled());
    ASSERT_FALSE(slamNode_->publishPoseIsEnabled());
    ASSERT_FALSE(slamNode_->mappingIsEnabled());
  }

  // Publish pose and Mapping enabling. Localization disabled.
  {
    disableAllFunctionality();
    ASSERT_FALSE(slamNode_->togglePublishPose(true));
    ASSERT_FALSE(slamNode_->toggleMapping(true));

    ASSERT_FALSE(slamNode_->localizationIsEnabled());
    ASSERT_FALSE(slamNode_->publishPoseIsEnabled());
    ASSERT_FALSE(slamNode_->mappingIsEnabled());
  }

  // Localization and Mapping enabling. Publish pose disabled.
  {
    disableAllFunctionality();
    ASSERT_TRUE(slamNode_->toggleLocalization(true));
    ASSERT_TRUE(slamNode_->toggleMapping(true));

    ASSERT_TRUE(slamNode_->localizationIsEnabled());
    ASSERT_FALSE(slamNode_->publishPoseIsEnabled());
    ASSERT_TRUE(slamNode_->mappingIsEnabled());
  }

  // Localization, publish pose and Mapping enabling.
  {
    enableAllFunctionality();

    ASSERT_TRUE(slamNode_->localizationIsEnabled());
    ASSERT_TRUE(slamNode_->publishPoseIsEnabled());
    ASSERT_TRUE(slamNode_->mappingIsEnabled());
  }
}

TEST_F(SlamNodeSetupTest, ClearMap) {  // NOLINT
  enableAllFunctionality();
  slamNode_->clearMap();

  ASSERT_FALSE(slamNode_->localizationIsEnabled());
  ASSERT_FALSE(slamNode_->publishPoseIsEnabled());
  ASSERT_FALSE(slamNode_->mappingIsEnabled());
}

TEST_F(SlamNodeSetupTest, Shutdown) {  // NOLINT
  enableAllFunctionality();
  slamNode_->shutdown();

  ASSERT_TRUE(slamNode_->shutdownRequested());
}

}  // namespace icp_tools_global_mapping
