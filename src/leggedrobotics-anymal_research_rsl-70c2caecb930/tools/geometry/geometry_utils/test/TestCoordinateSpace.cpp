/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Tests CoordinateSpace
 */

#include <gtest/gtest.h>

#include <kindr/common/gtest_eigen.hpp>

#include <geometry_utils/geometry_utils.hpp>

namespace geometry_utils {

class TestPointOnLineSegment : public ::testing::Test {
 public:
  Eigen::Vector3d startPoint_{1.0, 5.0, 2.0};
  Eigen::Vector3d endPoint_{4.0, 9.0, 5.0};
  double epsilon_ = 1e-10;
};

class TestClosestPointOnLineSegment : public ::testing::Test {
 public:
  Eigen::Vector3d startPoint_{1.0, 5.0, 2.0};
  Eigen::Vector3d endPoint_{4.0, 9.0, 5.0};
  double epsilon_ = 1e-10;
};

class TestClosestPoseOnLineSegment : public ::testing::Test {
 public:
  PoseStamped startPose_{"map", Time(1.2), Position{}, RotationQuaternion{}};
  PoseStamped endPose_{"map", Time(1.2), Position{}, RotationQuaternion{}};
  double epsilon_ = 1e-10;
};

TEST_F(TestPointOnLineSegment, beforeStartWithinTolerance) {  // NOLINT
  Eigen::Vector3d point{0.994855, 4.99314, 1.99486};
  EXPECT_TRUE(CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.011));
}

TEST_F(TestPointOnLineSegment, beforeStartOutsideTolerance) {  // NOLINT
  Eigen::Vector3d point{0.994855, 4.99314, 1.99486};
  EXPECT_FALSE(CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.009));
}

TEST_F(TestPointOnLineSegment, midPointWithinTolerance) {  // NOLINT
  Eigen::Vector3d point{2.49, 7, 3.5};                     // Distance to closest point on line 0.00857493
  EXPECT_TRUE(CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.009));
}

TEST_F(TestPointOnLineSegment, midPointOutsideTolerance) {  // NOLINT
  Eigen::Vector3d point{2.49, 7, 3.5};                      // Distance to closest point on line 0.00857493
  EXPECT_FALSE(CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.008));
}

TEST_F(TestPointOnLineSegment, afterEndwWithinTolerance) {  // NOLINT
  Eigen::Vector3d point{4.00514, 9.00686, 5.00514};
  EXPECT_TRUE(CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.011));
}

TEST_F(TestPointOnLineSegment, afterEndOutsideTolerance) {  // NOLINT
  Eigen::Vector3d point{4.00514, 9.00686, 5.00514};
  EXPECT_FALSE(CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.009));
}

TEST_F(TestClosestPointOnLineSegment, stopsAtStart) {  // NOLINT
  Eigen::Vector3d point{0.5, 4.5, 1.5};

  auto closestPointOnLine =
      CoordinateSpace::calculateClosestPointInfoOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_);
  EXPECT_NEAR(closestPointOnLine.distance_, 0.866025403784439, epsilon_);
  EXPECT_EQ(closestPointOnLine.isCloserToStart_, true);
  EXPECT_TRUE(closestPointOnLine.point_.isApprox(startPoint_, epsilon_));
}

TEST_F(TestClosestPointOnLineSegment, stopsAtEnd) {  // NOLINT
  Eigen::Vector3d point{4.5, 9.5, 5.5};
  auto closestPointOnLine =
      CoordinateSpace::calculateClosestPointInfoOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_);
  EXPECT_NEAR(closestPointOnLine.distance_, 0.866025403784439, epsilon_);
  EXPECT_EQ(closestPointOnLine.isCloserToStart_, false);
  EXPECT_TRUE(closestPointOnLine.point_.isApprox(endPoint_, epsilon_));
}

TEST_F(TestClosestPointOnLineSegment, findsClosestPointOnLine) {  // NOLINT
  Eigen::Vector3d point{2.5, 6.5, 1.5};
  double distance =
      CoordinateSpace::calculateClosestPointInfoOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_).distance_;
  EXPECT_NEAR(distance, 1.53871604229745, epsilon_);
}

TEST_F(TestClosestPoseOnLineSegment, findsUnWeightedClosestPose) {  // NOLINT
  endPose_.position_ = Position{1, 0, 1};
  PoseStamped referencePose = startPose_;
  referencePose.position_ = Position{1, 0, 0};

  Vector4D weights = Vector4D::Ones();
  auto info =
      CoordinateSpace::calculateWeightedClosestPoseInfoOnLineSegment<DimensionType::Txyz>(referencePose, startPose_, endPose_, weights);

  EXPECT_TRUE(info.success_);
  EXPECT_NEAR(info.distance_, sqrt(2.0) / 2.0, epsilon_);
  KINDR_ASSERT_DOUBLE_MX_EQ(info.pose_.position_.toImplementation(), Eigen::Vector3d(0.5, 0.0, 0.5), 0.01, "");
}

TEST_F(TestClosestPoseOnLineSegment, findsWeightedZClosestPose) {  // NOLINT
  endPose_.position_ = Position{1, 0, 1};
  PoseStamped referencePose = startPose_;
  referencePose.position_ = Position{1, 0, 0};

  Vector4D weights = Vector4D::Ones();
  // x distance "costs" more
  weights(0) = sqrt(3);

  auto info =
      CoordinateSpace::calculateWeightedClosestPoseInfoOnLineSegment<DimensionType::Txyz>(referencePose, startPose_, endPose_, weights);

  EXPECT_TRUE(info.success_);
  EXPECT_FALSE(info.isCloserToStart_);
  EXPECT_NEAR(info.distance_, 0.86602540378, epsilon_);
  KINDR_ASSERT_DOUBLE_MX_EQ(info.pose_.position_.toImplementation(), Eigen::Vector3d(0.75, 0.0, 0.75), 0.01, "");

  // Make sure we found the sweet spot
  EXPECT_LT(info.distance_, CoordinateSpace::getWeightedDistanceBetweenPoints<TranslationDimensionType::Txyz>(
                                referencePose.position_.toImplementation(), Vector3D(0.76, 0.0, 0.76), weights.topRows<3>()));
  EXPECT_LT(info.distance_, CoordinateSpace::getWeightedDistanceBetweenPoints<TranslationDimensionType::Txyz>(
                                referencePose.position_.toImplementation(), Vector3D(0.74, 0.0, 0.74), weights.topRows<3>()));
}

TEST_F(TestClosestPoseOnLineSegment, findWeightedYawClosestPoint) {  // NOLINT
  PoseStamped referencePose = startPose_;
  referencePose.orientation_ = EulerAnglesZyx(0.01, 0.0, 0.0);
  startPose_.orientation_ = EulerAnglesZyx(3.0 * M_PI_4, 0.0, 0.0);
  endPose_.orientation_ = EulerAnglesZyx(-3.0 * M_PI_4, 0.0, 0.0);

  // Check first direction for reference pose closer to start outside of the arc connecting start and end.
  auto info = CoordinateSpace::calculateWeightedClosestPoseInfoOnLineSegment<DimensionType::TxyzRy>(referencePose, startPose_, endPose_,
                                                                                                    Vector4D::Ones());
  ASSERT_TRUE(info.pose_.orientation_.isNear(startPose_.orientation_, epsilon_));

  // Check opposite direction for reference pose closer to start outside of the arc connecting start and end.
  info = CoordinateSpace::calculateWeightedClosestPoseInfoOnLineSegment<DimensionType::TxyzRy>(referencePose, endPose_, startPose_,
                                                                                               Vector4D::Ones());
  ASSERT_TRUE(info.pose_.orientation_.isNear(startPose_.orientation_, epsilon_));

  // Check first direction for reference pose closer to end outside of the arc connecting start and end.
  referencePose.orientation_ = EulerAnglesZyx(-0.01, 0.0, 0.0);
  info = CoordinateSpace::calculateWeightedClosestPoseInfoOnLineSegment<DimensionType::TxyzRy>(referencePose, startPose_, endPose_,
                                                                                               Vector4D::Ones());
  ASSERT_TRUE(info.pose_.orientation_.isNear(endPose_.orientation_, epsilon_));

  // Check opposite direction for reference pose closer to end outside of the arc connecting start and end.
  info = CoordinateSpace::calculateWeightedClosestPoseInfoOnLineSegment<DimensionType::TxyzRy>(referencePose, endPose_, startPose_,
                                                                                               Vector4D::Ones());
  ASSERT_TRUE(info.pose_.orientation_.isNear(endPose_.orientation_, epsilon_));

  // Check for reference pose inside of the arc connecting start and end.
  referencePose.orientation_ = EulerAnglesZyx(M_PI, 0.0, 0.0);
  info = CoordinateSpace::calculateWeightedClosestPoseInfoOnLineSegment<DimensionType::TxyzRy>(referencePose, startPose_, endPose_,
                                                                                               Vector4D::Ones());
  ASSERT_TRUE(info.pose_.orientation_.isNear(referencePose.orientation_, epsilon_));
  info = CoordinateSpace::calculateWeightedClosestPoseInfoOnLineSegment<DimensionType::TxyzRy>(referencePose, endPose_, startPose_,
                                                                                               Vector4D::Ones());
  ASSERT_TRUE(info.pose_.orientation_.isNear(referencePose.orientation_, epsilon_));
}

}  // namespace geometry_utils
