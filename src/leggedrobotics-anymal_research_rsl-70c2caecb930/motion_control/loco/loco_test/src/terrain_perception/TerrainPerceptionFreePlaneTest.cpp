/*
 * TerrainPerceptionFreePlaneTest.cpp
 *
 *  Created on: May 7, 2015
 *      Author: Dario Bellicoso, Peter Fankhauser
 */


#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"
#include <gtest/gtest.h>

TEST(TerrainPerceptionFreePlaneTest, testModelGeneration)
{

  loco::TerrainModelFreePlane terrainModel;
  std::vector<loco::Position> pointsInWorldFrame;

  pointsInWorldFrame.push_back(loco::Position(0.0, 0.0, 0.0));
  pointsInWorldFrame.push_back(loco::Position(0.0, 1.0, 0.0));
  pointsInWorldFrame.push_back(loco::Position(1.0, 0.0, 0.0));
  pointsInWorldFrame.push_back(loco::Position(1.0, 1.0, 0.0));

  loco::TerrainPerceptionFreePlane::generateTerrainModelFromPointsInWorldFrame(pointsInWorldFrame,
                                                                               terrainModel);
}

TEST(TerrainPerceptionFreePlaneTest, testSurfaceNormal)
{

  loco::TerrainModelFreePlane terrainModel;
  std::vector<loco::Position> pointsInWorldFrame;

  pointsInWorldFrame.push_back(loco::Position(0.0, -1.0, 0.0));
  pointsInWorldFrame.push_back(loco::Position(0.0,  1.0, 0.0));
  pointsInWorldFrame.push_back(loco::Position(1.0, 0.0, 1.0));

  loco::TerrainPerceptionFreePlane::generateTerrainModelFromPointsInWorldFrame(pointsInWorldFrame,
                                                                               terrainModel);
  loco::Vector normal;
  terrainModel.getNormal(loco::Position(0.0, 0.0, 0.0), normal);

  EXPECT_NEAR(-0.70710678118, normal.x(), 0.0001);
  EXPECT_NEAR(0.0, normal.y(), 0.0001);
  EXPECT_NEAR(0.70710678118, normal.z(), 0.0001);
}
