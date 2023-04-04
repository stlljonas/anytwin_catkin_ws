/**
 * @authors     Prajish Sankar
 * @affiliation ANYbotics
 * @date        Jun 29, 2020
 * @brief       Tests for VelocityClipping struct
 */

// gtest
#include <gtest/gtest.h>

// kindr
#include <kindr/common/gtest_eigen.hpp>

// motion generation
#include "motion_generation_utils/VelocityClipping.hpp"

TEST(VelocityClippingTests, BoxClipping) {
  using namespace std;

  const Eigen::Vector3d maxLinearVelocity(0.5 *
                                          (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()));  // making the range of max vel = [0, 1].

  for (unsigned int testId = 0u; testId < 10u; ++testId) {
    Eigen::Vector3d desiredLinearVelocity(Eigen::Vector3d::Random());  // in range [-1, 1].
    motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);

    EXPECT_TRUE((desiredLinearVelocity.x() >= -abs(maxLinearVelocity.x())) && (desiredLinearVelocity.x() <= abs(maxLinearVelocity.x())));
    EXPECT_TRUE((desiredLinearVelocity.y() >= -abs(maxLinearVelocity.y())) && (desiredLinearVelocity.y() <= abs(maxLinearVelocity.y())));
    EXPECT_TRUE((desiredLinearVelocity.z() >= -abs(maxLinearVelocity.z())) && (desiredLinearVelocity.z() <= abs(maxLinearVelocity.z())));
  }
}

TEST(VelocityClippingTests, EllipsoidClipping) {
  using namespace std;

  const Eigen::Vector3d maxLinearVelocity(0.5 *
                                          (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()));  // making the range of max vel = [0, 1].

  for (unsigned int testId = 0u; testId < 10u; ++testId) {
    Eigen::Vector3d desiredLinearVelocity(Eigen::Vector3d::Random());  // in range [-1, 1].

    Eigen::Vector3d desiredLinearVelocityCopy = desiredLinearVelocity;
    const double velocityMagnitudeBefore = desiredLinearVelocity.norm();
    motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
    const double velocityMagnitudeAfter = desiredLinearVelocity.norm();

    EXPECT_LE(velocityMagnitudeAfter, velocityMagnitudeBefore);
    if (std::abs(velocityMagnitudeBefore - velocityMagnitudeAfter) < 1e-10) {
      EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), desiredLinearVelocityCopy.x());
      EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), desiredLinearVelocityCopy.y());
      EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), desiredLinearVelocityCopy.z());
    }
  }
}

TEST(VelocityClippingTests, EllipsoidClippingExtremeCases) {
  using namespace std;
  constexpr double maxDouble = 1.0e150;
  constexpr double minDouble = -1.0e150;

  //! CASE Zero desired velocity
  Eigen::Vector3d maxLinearVelocity(0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()));  // making the range of max vel = [0, 1].
  Eigen::Vector3d desiredLinearVelocity(Eigen::Vector3d::Zero());
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  //! CASE Desired velocity in one direction only
  maxLinearVelocity = 0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones());  // making the range of max vel = [0, 1].

  desiredLinearVelocity << maxDouble, 0.0, 0.0;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << minDouble, 0.0, 0.0;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), -maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << 0.0, maxDouble, 0.0;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << 0.0, minDouble, 0.0;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), -maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << 0.0, 0.0, maxDouble;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), maxLinearVelocity.z());

  desiredLinearVelocity << 0.0, 0.0, minDouble;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), -maxLinearVelocity.z());

  //! CASE Desired velocity in x,y directions only
  maxLinearVelocity = 0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones());  // making the range of max vel = [0, 1].
  const double a = 1.0 / std::sqrt(2);

  desiredLinearVelocity << maxDouble, maxDouble, 0.0;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), maxLinearVelocity.x() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), maxLinearVelocity.y() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << maxDouble, minDouble, 0.0;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), maxLinearVelocity.x() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), -maxLinearVelocity.y() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << minDouble, maxDouble, 0.0;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), -maxLinearVelocity.x() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), maxLinearVelocity.y() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << minDouble, minDouble, 0.0;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), -maxLinearVelocity.x() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), -maxLinearVelocity.y() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  //! CASE Desired velocity in x,z directions only
  maxLinearVelocity = 0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones());  // making the range of max vel = [0, 1].

  desiredLinearVelocity << maxDouble, 0.0, maxDouble;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), maxLinearVelocity.x() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), maxLinearVelocity.z() * a);

  desiredLinearVelocity << maxDouble, 0.0, minDouble;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), maxLinearVelocity.x() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), -maxLinearVelocity.z() * a);

  desiredLinearVelocity << minDouble, 0.0, maxDouble;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), -maxLinearVelocity.x() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), maxLinearVelocity.z() * a);

  desiredLinearVelocity << minDouble, 0.0, minDouble;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), -maxLinearVelocity.x() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), -maxLinearVelocity.z() * a);

  //! CASE Desired velocity in y,z directions only
  maxLinearVelocity = 0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones());  // making the range of max vel = [0, 1].

  desiredLinearVelocity << 0.0, maxDouble, maxDouble;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), maxLinearVelocity.y() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), maxLinearVelocity.z() * a);

  desiredLinearVelocity << 0.0, maxDouble, minDouble;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), maxLinearVelocity.y() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), -maxLinearVelocity.z() * a);

  desiredLinearVelocity << 0.0, minDouble, maxDouble;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), -maxLinearVelocity.y() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), maxLinearVelocity.z() * a);

  desiredLinearVelocity << 0.0, minDouble, minDouble;
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), -maxLinearVelocity.y() * a);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), -maxLinearVelocity.z() * a);

  //! CASE max velocities are negative
  maxLinearVelocity = -0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones());  // making the range of max vel = [-1, 0].
  desiredLinearVelocity = Eigen::Vector3d::Random();                                 // in range [-1, 1].

  Eigen::Vector3d desiredLinearVelocityCopy = desiredLinearVelocity;
  const double velocityMagnitudeBefore = desiredLinearVelocity.norm();
  motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity);
  const double velocityMagnitudeAfter = desiredLinearVelocity.norm();

  EXPECT_LE(velocityMagnitudeAfter, velocityMagnitudeBefore);
  if (std::abs(velocityMagnitudeBefore - velocityMagnitudeAfter) < 1e-10) {
    EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), desiredLinearVelocityCopy.x());
    EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), desiredLinearVelocityCopy.y());
    EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), desiredLinearVelocityCopy.z());
  }
}

TEST(VelocityClippingTests, BoxClippingExtremeCases) {
  using namespace std;
  constexpr double maxDouble = 1.0e150;
  constexpr double minDouble = -1.0e150;

  //! CASE Zero desired velocity
  Eigen::Vector3d maxLinearVelocity(0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()));  // making the range of max vel = [0, 1].
  Eigen::Vector3d desiredLinearVelocity(Eigen::Vector3d::Zero());
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  //! CASE Desired velocity in one direction only
  maxLinearVelocity = 0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones());  // making the range of max vel = [0, 1].

  desiredLinearVelocity << maxDouble, 0.0, 0.0;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << minDouble, 0.0, 0.0;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), -maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << 0.0, maxDouble, 0.0;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << 0.0, minDouble, 0.0;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), -maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << 0.0, 0.0, maxDouble;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), maxLinearVelocity.z());

  desiredLinearVelocity << 0.0, 0.0, minDouble;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), -maxLinearVelocity.z());

  //! CASE Desired velocity in x,y directions only
  maxLinearVelocity = 0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones());  // making the range of max vel = [0, 1].

  desiredLinearVelocity << maxDouble, maxDouble, 0.0;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << maxDouble, minDouble, 0.0;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), -maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << minDouble, maxDouble, 0.0;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), -maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  desiredLinearVelocity << minDouble, minDouble, 0.0;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), -maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), -maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), 0.0);

  //! CASE Desired velocity in x,z directions only
  maxLinearVelocity = 0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones());  // making the range of max vel = [0, 1].

  desiredLinearVelocity << maxDouble, 0.0, maxDouble;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), maxLinearVelocity.z());

  desiredLinearVelocity << maxDouble, 0.0, minDouble;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), -maxLinearVelocity.z());

  desiredLinearVelocity << minDouble, 0.0, maxDouble;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), -maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), maxLinearVelocity.z());

  desiredLinearVelocity << minDouble, 0.0, minDouble;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), -maxLinearVelocity.x());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), -maxLinearVelocity.z());

  //! CASE Desired velocity in y,z directions only
  maxLinearVelocity = 0.5 * (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones());  // making the range of max vel = [0, 1].

  desiredLinearVelocity << 0.0, maxDouble, maxDouble;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), maxLinearVelocity.z());

  desiredLinearVelocity << 0.0, maxDouble, minDouble;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), -maxLinearVelocity.z());

  desiredLinearVelocity << 0.0, minDouble, maxDouble;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), -maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), maxLinearVelocity.z());

  desiredLinearVelocity << 0.0, minDouble, minDouble;
  motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.x(), 0.0);
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.y(), -maxLinearVelocity.y());
  EXPECT_DOUBLE_EQ(desiredLinearVelocity.z(), -maxLinearVelocity.z());
}

TEST(VelocityClippingTests, BoxClippingWithVelocityOffset) {
  using namespace std;

  const Eigen::Vector3d maxLinearVelocity(0.5 *
                                          (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()));  // making the range of max vel = [0, 1].

  for (unsigned int testId = 0u; testId < 10u; ++testId) {
    Eigen::Vector3d desiredLinearVelocity(Eigen::Vector3d::Random());  // in range [-1, 1].
    Eigen::Vector2d velocityOffset(Eigen::Vector2d::Random());         // in range [-1, 1].
    motion_generation_utils::clipToBox(desiredLinearVelocity, maxLinearVelocity, velocityOffset);

    if (abs(velocityOffset.x()) <= abs(maxLinearVelocity.x())) {
      EXPECT_TRUE((desiredLinearVelocity.x() >= -abs(maxLinearVelocity.x()) + velocityOffset.x()) &&
                  (desiredLinearVelocity.x() <= abs(maxLinearVelocity.x()) + velocityOffset.x()));
    } else if (velocityOffset.x() > maxLinearVelocity.x()) {
      EXPECT_TRUE((desiredLinearVelocity.x() >= 0.0) && (desiredLinearVelocity.x() <= abs(maxLinearVelocity.x()) + velocityOffset.x()));
    } else {
      EXPECT_TRUE((desiredLinearVelocity.x() >= -abs(maxLinearVelocity.x()) + velocityOffset.x()) && (desiredLinearVelocity.x() <= 0.0));
    }

    if (abs(velocityOffset.y()) <= abs(maxLinearVelocity.y())) {
      EXPECT_TRUE((desiredLinearVelocity.y() >= -abs(maxLinearVelocity.y()) + velocityOffset.y()) &&
                  (desiredLinearVelocity.y() <= abs(maxLinearVelocity.y()) + velocityOffset.y()));
    } else if (velocityOffset.y() > maxLinearVelocity.y()) {
      EXPECT_TRUE((desiredLinearVelocity.y() >= 0.0) && (desiredLinearVelocity.y() <= abs(maxLinearVelocity.y()) + velocityOffset.y()));
    } else {
      EXPECT_TRUE((desiredLinearVelocity.y() >= -abs(maxLinearVelocity.y()) + velocityOffset.y()) && (desiredLinearVelocity.y() <= 0.0));
    }

    EXPECT_TRUE((desiredLinearVelocity.z() >= -abs(maxLinearVelocity.z())) && (desiredLinearVelocity.z() <= abs(maxLinearVelocity.z())));
  }
}

TEST(VelocityClippingTests, EllipsoidClippingWithVelocityOffset) {
  using namespace std;

  const Eigen::Vector3d maxLinearVelocity(0.5 *
                                          (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()));  // making the range of max vel = [0, 1].

  for (unsigned int testId = 0u; testId < 10u; ++testId) {
    Eigen::Vector3d desiredLinearVelocity(Eigen::Vector3d::Random());  // in range [-1, 1].
    Eigen::Vector2d velocityOffset(Eigen::Vector2d::Random());         // in range [-1, 1].

    Eigen::Vector3d desiredLinearVelocityCopy = desiredLinearVelocity;
    const double velocityMagnitudeBefore = desiredLinearVelocity.norm();
    Eigen::Vector3d desiredLineatVelocityAssumingNoOffset = desiredLinearVelocity;
    Eigen::Vector3d desiredLineatVelocityAssumingNoOffsetCopy = desiredLineatVelocityAssumingNoOffset;

    motion_generation_utils::clipToEllipsoid(desiredLineatVelocityAssumingNoOffset, maxLinearVelocity);  // Helper

    motion_generation_utils::clipToEllipsoid(desiredLinearVelocity, maxLinearVelocity, velocityOffset);
    const double velocityMagnitudeAfter = desiredLinearVelocity.norm();

    EXPECT_LE(velocityMagnitudeAfter, velocityMagnitudeBefore);
    if (velocityMagnitudeBefore < velocityMagnitudeAfter) {  // Case of no clipping

      if (abs(velocityOffset.x()) <= abs(maxLinearVelocity.x())) {
        EXPECT_TRUE((desiredLinearVelocity.x() >= -abs(maxLinearVelocity.x()) + velocityOffset.x()) &&
                    (desiredLinearVelocity.x() <= abs(maxLinearVelocity.x()) + velocityOffset.x()));
      } else if (velocityOffset.x() > maxLinearVelocity.x()) {
        EXPECT_TRUE((desiredLinearVelocity.x() >= 0.0) && (desiredLinearVelocity.x() <= abs(maxLinearVelocity.x()) + velocityOffset.x()));
      } else {
        EXPECT_TRUE((desiredLinearVelocity.x() >= -abs(maxLinearVelocity.x()) + velocityOffset.x()) && (desiredLinearVelocity.x() <= 0.0));
      }

      if (abs(velocityOffset.y()) <= abs(maxLinearVelocity.y())) {
        EXPECT_TRUE((desiredLinearVelocity.y() >= -abs(maxLinearVelocity.y()) + velocityOffset.y()) &&
                    (desiredLinearVelocity.y() <= abs(maxLinearVelocity.y()) + velocityOffset.y()));
      } else if (velocityOffset.y() > maxLinearVelocity.y()) {
        EXPECT_TRUE((desiredLinearVelocity.y() >= 0.0) && (desiredLinearVelocity.y() <= abs(maxLinearVelocity.y()) + velocityOffset.y()));
      } else {
        EXPECT_TRUE((desiredLinearVelocity.y() >= -abs(maxLinearVelocity.y()) + velocityOffset.y()) && (desiredLinearVelocity.y() <= 0.0));
      }

      EXPECT_TRUE((desiredLinearVelocity.z() >= -abs(maxLinearVelocity.z())) && (desiredLinearVelocity.z() <= abs(maxLinearVelocity.z())));

    } else if (abs(velocityMagnitudeAfter - velocityMagnitudeBefore) <= 1e-10) {  // Case of clipping
      EXPECT_TRUE(desiredLinearVelocity.x() <= desiredLineatVelocityAssumingNoOffset.x() + velocityOffset.x());
      EXPECT_TRUE(desiredLinearVelocity.y() <= desiredLineatVelocityAssumingNoOffset.y() + velocityOffset.y());
      EXPECT_TRUE(desiredLinearVelocity.z() <= desiredLineatVelocityAssumingNoOffset.z());
    }
  }
}