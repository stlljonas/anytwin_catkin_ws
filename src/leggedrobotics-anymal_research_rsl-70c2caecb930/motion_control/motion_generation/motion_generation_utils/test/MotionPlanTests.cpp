/*!
* @file     MotionPlanTests.cpp
* @author   Dario Bellicoso
* @date     Feb 8, 2018
* @brief
*/

// gtest
#include <gtest/gtest.h>

// kindr
#include <kindr/common/gtest_eigen.hpp>

// robot utils
#include <robot_utils/math/math.hpp>

// motion generation
#include "motion_generation_utils/motion_generation.hpp"
#include "motion_generation_utils/typedefs.hpp"
#include "motion_generation_utils/conversions.hpp"

TEST(MotionPlanTests, AngularVelocity1) {
  using namespace motion_generation;

  const EulerAnglesZyx orientationWorldToBody = EulerAnglesZyx(Eigen::Vector3d::Random());
  const EulerAnglesZyx orientationBodyToWorld = orientationWorldToBody.inverted().getUnique();
  const EulerAnglesZyxDiff eulerRatesBodyZyxInBodyFrame = EulerAnglesZyxDiff(Eigen::Vector3d::Random());

  // Compute angular velocity method 1.
  const LocalAngularVelocity angularVelocityBodyInWorldFrame = LocalAngularVelocity(
      zmp::getMapEulerAnglesZyxToAngularVelocityInInertialFrame(orientationBodyToWorld)*
      eulerRatesBodyZyxInBodyFrame.toImplementation()
  );

  // Compute angular velocity method 2.
  const LocalAngularVelocity angularVelocityBodyInBodyFrame = LocalAngularVelocity(
      zmp::getMapEulerAnglesZyxToAngularVelocityInBodyFrame(orientationBodyToWorld)*
      eulerRatesBodyZyxInBodyFrame.toImplementation()
  );

  const LocalAngularVelocity angularVelocityBodyInWorldFrame2 = orientationWorldToBody.inverseRotate(angularVelocityBodyInBodyFrame);

  // Check if identical.
  KINDR_ASSERT_DOUBLE_MX_EQ_ZT(angularVelocityBodyInWorldFrame.toImplementation(),
                               angularVelocityBodyInWorldFrame2.toImplementation(), 1.0, "", 1.0e-7);
}

TEST(MotionPlanTests, AngularVelocity2) {
  using namespace motion_generation;

  const EulerAnglesZyx orientationWorldToBody(1.0, -0.5, -0.2);
  const EulerAnglesZyx orientationBodyToWorld = orientationWorldToBody.inverted().getUnique();
  const LocalAngularVelocity angularVelocityInBodyFrame(0.1, 0.2, -0.5);

  // Compute angular rates method 1.
  auto angularRatesTargetInWorldFrame = EulerAnglesZyxDiff(
      zmp::getMatrixAngularVelocityInBodyFrameToAngularRatesZyx(orientationBodyToWorld) *
      angularVelocityInBodyFrame.toImplementation()
  );

  // Compute angular rates method 2.
  auto angularRatesTargetInWorldFrame2 = EulerAnglesZyxDiff(
      zmp::getMatrixAngularVelocityInInertialFrameToAngularRatesZyx(orientationBodyToWorld) *
      orientationWorldToBody.inverseRotate(angularVelocityInBodyFrame).toImplementation()
  );

  // Check if identical.
  EXPECT_NEAR(angularRatesTargetInWorldFrame.x(), angularRatesTargetInWorldFrame2.x(), 1e-6);
  EXPECT_NEAR(angularRatesTargetInWorldFrame.y(), angularRatesTargetInWorldFrame2.y(), 1e-6);
  EXPECT_NEAR(angularRatesTargetInWorldFrame.z(), angularRatesTargetInWorldFrame2.z(), 1e-6);
}

TEST(MotionPlanTests, AngularAcceleration) {
  using namespace motion_generation;

  EulerAnglesZyx       orientationWorldToBody(1.0, -0.5, -0.2);
  EulerAnglesZyx       orientationBodyToWorld = orientationWorldToBody.inverted().getUnique();
  LocalAngularVelocity angularVelocityInBodyFrame(0.1, 0.2, -0.5);
  AngularAcceleration  angularAccelerationInBodyFrame(-0.2, 0.1, -0.1);

  // Convert to euler derivative.
  const EulerAnglesZyxDiff eulerRatesBodyZyxInWorldFrame = EulerAnglesZyxDiff(
      zmp::getMatrixAngularVelocityInBodyFrameToAngularRatesZyx(orientationBodyToWorld)*
      angularVelocityInBodyFrame.toImplementation()
  );

  const EulerAnglesZyxDiff eulerAccelerationBodyZyxInWorldFrame = EulerAnglesZyxDiff(
      zmp::getMatrixAngularVelocityInBodyFrameToAngularRatesZyxTimeDerivative(orientationBodyToWorld, eulerRatesBodyZyxInWorldFrame)*
      angularVelocityInBodyFrame.toImplementation() +
      zmp::getMatrixAngularVelocityInBodyFrameToAngularRatesZyx(orientationBodyToWorld)*
      angularAccelerationInBodyFrame.toImplementation()
  );

  // Convert back to angular derivative.
  const LocalAngularVelocity angularVelocityInBodyFrame2 = LocalAngularVelocity(orientationBodyToWorld, eulerRatesBodyZyxInWorldFrame);
//  const LocalAngularVelocity angularVelocityInBodyFrame2 = LocalAngularVelocity(
//      zmp::getMapEulerAnglesZyxToAngularVelocityInBodyFrame(orientationBodyToWorld)*
//      eulerRatesBodyZyxInWorldFrame.toImplementation()
//  );

  AngularAcceleration angularAccelerationInBodyFrame2 =
      zmp::convertEulerAnglesZYXAccelerationToAngularAccelerationInBodyFrame(
          orientationBodyToWorld, eulerRatesBodyZyxInWorldFrame, eulerAccelerationBodyZyxInWorldFrame);

  // Check if identical.
  EXPECT_NEAR(angularVelocityInBodyFrame.x(), angularVelocityInBodyFrame2.x(), 1e-6);
  EXPECT_NEAR(angularVelocityInBodyFrame.y(), angularVelocityInBodyFrame2.y(), 1e-6);
  EXPECT_NEAR(angularVelocityInBodyFrame.z(), angularVelocityInBodyFrame2.z(), 1e-6);

  EXPECT_NEAR(angularAccelerationInBodyFrame.x(), angularAccelerationInBodyFrame2.x(), 1e-6);
  EXPECT_NEAR(angularAccelerationInBodyFrame.y(), angularAccelerationInBodyFrame2.y(), 1e-6);
  EXPECT_NEAR(angularAccelerationInBodyFrame.z(), angularAccelerationInBodyFrame2.z(), 1e-6);
}

TEST(MotionPlanTests, Indexing) {
  using namespace motion_generation;

  EulerAnglesZyx orientationWorldToBody(0.1, 0.2, 0.3);
  EXPECT_NEAR(orientationWorldToBody.roll(),  orientationWorldToBody.toImplementation()(zmp::toIndex(zmp::CogDim::roll)),  1e-6);
  EXPECT_NEAR(orientationWorldToBody.pitch(), orientationWorldToBody.toImplementation()(zmp::toIndex(zmp::CogDim::pitch)), 1e-6);
  EXPECT_NEAR(orientationWorldToBody.yaw(),   orientationWorldToBody.toImplementation()(zmp::toIndex(zmp::CogDim::yaw)),   1e-6);

  LinearVelocity linearVeloccityInWorldFrame(0.1, 0.2, 0.3);
  EXPECT_NEAR(linearVeloccityInWorldFrame.x(), linearVeloccityInWorldFrame.toImplementation()(zmp::toIndex(zmp::CogDim::x)), 1e-6);
  EXPECT_NEAR(linearVeloccityInWorldFrame.y(), linearVeloccityInWorldFrame.toImplementation()(zmp::toIndex(zmp::CogDim::y)), 1e-6);
  EXPECT_NEAR(linearVeloccityInWorldFrame.z(), linearVeloccityInWorldFrame.toImplementation()(zmp::toIndex(zmp::CogDim::z)), 1e-6);
}

TEST(MotionPlanTests, AverageRotationQuaternion1) {
  using namespace motion_generation;

  const EulerAnglesZyx R1zyx(1.0, -0.5, -0.2);
  auto R1 = RotationQuaternion(R1zyx);
  auto R2 = R1;
  const Vector vector(-0.2, 2.4, 0.8);

  robot_utils::alphaFilter(R1, R2, 0.8, 0.2);

  const Vector vectorRotated1 =  R1.rotate(vector);
  const Vector vectorRotated2 =  R2.rotate(vector);

  EXPECT_NEAR(vectorRotated1.x(), vectorRotated2.x(), 1e-6);
  EXPECT_NEAR(vectorRotated1.y(), vectorRotated2.y(), 1e-6);
  EXPECT_NEAR(vectorRotated1.z(), vectorRotated2.z(), 1e-6);
}

TEST(MotionPlanTests, AverageRotationQuaternion2) {
  using namespace motion_generation;

  const double angle = 1.5;
  const Vector vector(-0.2, 2.4, 0.8);

  for (unsigned int id=0u; id<3u; ++id) {
    EulerAnglesZyx R1zyx(0.0, 0.0, 0.0);
    EulerAnglesZyx R2zyx(0.0, 0.0, 0.0);

    R1zyx.toImplementation()[id] = angle;
    R2zyx.toImplementation()[id] = -angle;

    auto R1 = RotationQuaternion(R1zyx);
    auto R2 = RotationQuaternion(R2zyx);


    // should yield unit rotation.
    robot_utils::alphaFilter(R1, R2, 0.5, 0.5);

    const Vector vectorRotated1 =  R1.rotate(vector);

    EXPECT_NEAR(vectorRotated1.x(), vector.x(), 1e-6);
    EXPECT_NEAR(vectorRotated1.y(), vector.y(), 1e-6);
    EXPECT_NEAR(vectorRotated1.z(), vector.z(), 1e-6);
  }
}
