/*!
* @file    MotionGenerationUtilsTest.cpp
* @author  Dario Bellicoso
* @date    Jan 19, 2016
*/

// gtest
#include <gtest/gtest.h>

// eigen
#include <Eigen/Core>
#include <Eigen/SparseCore>

// kindr
#include <kindr/common/gtest_eigen.hpp>

// motion generation
#include <motion_generation_utils/motion_generation.hpp>
#include <motion_generation_utils/VirtualPlaneFrameBase.hpp>

// stl
#include <string>


TEST(VirtualPlaneFrameTests, testInit) {
  motion_generation::VirtualPlaneFrameBase vpf;

  const auto& pose = vpf.getPosePlaneToWorld();

  Eigen::Vector3d position = pose.getPosition().toImplementation();
  Eigen::Vector4d rotation = pose.getRotation().vector();

  Eigen::Vector3d positionExpected = motion_generation::Position::Zero().toImplementation();
  Eigen::Vector4d rotationExpected = motion_generation::RotationQuaternion().vector();

  std::string msg = "";
  KINDR_ASSERT_DOUBLE_MX_EQ(position, positionExpected, 1.0, msg);
  KINDR_ASSERT_DOUBLE_MX_EQ(rotation, rotationExpected, 1.0, msg);


  Eigen::Vector3d normal = vpf.getPlaneNormalInWorldFrame().toImplementation();
  Eigen::Vector3d normalExpected = Eigen::Vector3d::UnitZ();
  KINDR_ASSERT_DOUBLE_MX_EQ(normal, normalExpected, 1.0, msg);
}


TEST(VirtualPlaneFrameTests, testProjection) {
  motion_generation::VirtualPlaneFrameBase vpf;
  std::string msg = "";

  // Project along vertical axis.
  motion_generation::Position positionWorldToPointInWorldFrame(0.0, 0.0, 5.0);
  motion_generation::Position projectedPosition =
      vpf.projectOntoVirtualPlaneInWorldFrame(positionWorldToPointInWorldFrame,
                                              motion_generation::Vector::UnitZ());
  motion_generation::Position projectedPositionExpected = motion_generation::Position(0.0, 0.0, 0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ(projectedPosition.toImplementation(), projectedPositionExpected.toImplementation(), 1.0, msg);

  // Project along x-z axis.
  positionWorldToPointInWorldFrame = motion_generation::Position(5.0, 0.0, 1.0);
  projectedPosition = vpf.projectOntoVirtualPlaneInWorldFrame(
      positionWorldToPointInWorldFrame,
      motion_generation::Vector(1.0, 0.0, 1.0));
  projectedPositionExpected = motion_generation::Position(4.0, 0.0, 0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ(projectedPosition.toImplementation(), projectedPositionExpected.toImplementation(), 1.0, msg);

  positionWorldToPointInWorldFrame = motion_generation::Position(5.0, 0.0, 2.0);
  projectedPosition = vpf.projectOntoVirtualPlaneInWorldFrame(
      positionWorldToPointInWorldFrame,
      motion_generation::Vector(1.0, 0.0, 1.0));
  projectedPositionExpected = motion_generation::Position(3.0, 0.0, 0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ(projectedPosition.toImplementation(), projectedPositionExpected.toImplementation(), 1.0, msg);

  // Project along

}