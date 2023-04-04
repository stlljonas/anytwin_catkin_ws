/**
 * @authors     Dario Bellicoso
 * @affiliation RSL
 * @brief       Terrain perception free plane tests.
 */

// gtest
#include <gtest/gtest.h>

// kindr
#include <kindr/common/gtest_eigen.hpp>

// eigen
#include <Eigen/Core>

// loco
#include <loco/terrain_perception/TerrainPerceptionFreePlane.hpp>
#include <loco/common/TerrainModelFreePlane.hpp>

// loco anymal
#include "loco_anymal/testing/CommonFixture.hpp"

namespace loco_anymal {

class TerrainPerceptionFreePlaneTests : public CommonFixture {};

TEST_F(TerrainPerceptionFreePlaneTests, testModelGeneration) {
  loco::TerrainModelFreePlane terrainModel;
  std::vector<loco::Position> pointsInWorldFrame{
      {loco::Position(0.0, 0.0, 0.0), loco::Position(0.0, 1.0, 0.0), loco::Position(1.0, 0.0, 0.0), loco::Position(1.0, 1.0, 0.0)}};

  EXPECT_TRUE(loco::TerrainPerceptionFreePlane::generateTerrainModelFromPointsInWorldFrame(pointsInWorldFrame, terrainModel));
}

TEST_F(TerrainPerceptionFreePlaneTests, testSurfaceNormal) {
  loco::TerrainModelFreePlane terrainModel;
  std::vector<loco::Position> pointsInWorldFrame{
      {loco::Position(0.0, -1.0, 0.0), loco::Position(0.0, 1.0, 0.0), loco::Position(1.0, 0.0, 1.0)}};

  EXPECT_TRUE(loco::TerrainPerceptionFreePlane::generateTerrainModelFromPointsInWorldFrame(pointsInWorldFrame, terrainModel));

  loco::Vector normal;
  terrainModel.getNormal(loco::Position(0.0, 0.0, 0.0), normal);

  EXPECT_NEAR(-0.70710678118, normal.x(), 0.0001);
  EXPECT_NEAR(0.0, normal.y(), 0.0001);
  EXPECT_NEAR(0.70710678118, normal.z(), 0.0001);
}

TEST_F(TerrainPerceptionFreePlaneTests, testNormalAndPosition) {
  loco::TerrainModelFreePlane terrainModel;
  std::vector<loco::Position> pointsInWorldFrame{
      {loco::Position(0.0, 0.0, 0.0), loco::Position(0.0, 1.0, 0.0), loco::Position(1.0, 0.0, 0.0)}};

  EXPECT_TRUE(loco::TerrainPerceptionFreePlane::generateTerrainModelFromPointsInWorldFrame(pointsInWorldFrame, terrainModel));

  loco::Vector normal;
  terrainModel.getNormal(loco::Position(), normal);

  loco::Position position;
  terrainModel.getHeight(position);

  KINDR_ASSERT_DOUBLE_MX_EQ_ZT(position.toImplementation(), Eigen::Vector3d::Zero(), 1.0, "", 1e-10);
  KINDR_ASSERT_DOUBLE_MX_EQ_ZT(normal.toImplementation(), Eigen::Vector3d::UnitZ(), 1.0, "", 1e-10);
}

} // namespace loco_anymal
