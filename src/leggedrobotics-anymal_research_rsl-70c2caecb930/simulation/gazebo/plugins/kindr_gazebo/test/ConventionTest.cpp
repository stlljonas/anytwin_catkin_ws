/*
 * convention.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: gech
 */

// test
#include <gtest/gtest.h>

// kindr
#include <kindr/Core>
#include <kindr/rotations/gtest_rotations.hpp>

// gazebo math implementation
#include <gazebo/gazebo_config.h>
#if (GAZEBO_MAJOR_VERSION >= 8)
#include <ignition/math/Pose3.hh>
#else
#include <gazebo/math/gzmath.hh>
#endif

namespace kindr {

#if (GAZEBO_MAJOR_VERSION >= 8)
  using Quaterniond = ignition::math::Quaterniond;
#else
  using Quaterniond = gazebo::math::Quaternion;
#endif

template<>
class RotationConversion<Quaterniond, Eigen::Vector3d, double> {
  typedef double Scalar;
  typedef Quaterniond Rotation;
 public:
  inline static void convertToOtherRotation(Rotation& rotation, const kindr::RotationQuaternion<Scalar>& quaternionIn) {
    // Implement quaternionOut = f(quaternionIn);
    rotation = Rotation(quaternionIn.w(), quaternionIn.x(), quaternionIn.y(), quaternionIn.z());
  }

  inline static void convertToKindr(kindr::RotationQuaternion<Scalar>& quaternion, const Rotation& quaternionIn) {
#if (GAZEBO_MAJOR_VERSION >= 8)
    quaternion = kindr::RotationQuaternion<Scalar>(quaternionIn.W(), quaternionIn.X(), quaternionIn.Y(), quaternionIn.Z());
#else
  quaternion = kindr::RotationQuaternion<Scalar>(quaternionIn.w, quaternionIn.x, quaternionIn.y, quaternionIn.z);
#endif
  }

  inline static void convertToOtherVelocityVector(Eigen::Vector3d& velocityOut, const Rotation& rot, const Eigen::Matrix<Scalar,3,1>& velocityIn) {
    // Implement velocityOut = g(velocityIn, rot);
    velocityOut = velocityIn;
  }

  inline static void getRotationMatrixFromRotation(Eigen::Matrix<Scalar,3,3>& rotationMatrix, const Rotation& rotation) {
    // Implement rotationMatrix = C(quaternion);
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Matrix3d gazeboRotationMatrix(rotation);
#else
    gazebo::math::Matrix3 gazeboRotationMatrix = rotation.GetAsMatrix3();
#endif

    for (unsigned int k=0; k<3; k++) {
      for (unsigned int h=0; h<3; h++) {
#if (GAZEBO_MAJOR_VERSION >= 8)
        rotationMatrix(k,h) = gazeboRotationMatrix(k,h);
#else
        rotationMatrix(k,h) = gazeboRotationMatrix[k][h];
#endif
      }
    }
  }

  inline static void concatenate(Rotation& res,  const Rotation& rot1,  const Rotation& rot2) {
    res = rot2*rot1;
  }

  inline static void rotateVector(Eigen::Matrix<Scalar,3,1>& A_r, const Rotation& rotationBToA, const Eigen::Matrix<Scalar,3,1>& B_r) {
    //RigidBodyDynamics::Math::SpatialVector vector()
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d A_v = rotationBToA.RotateVector(ignition::math::Vector3d(B_r.x(), B_r.y(), B_r.z()));
    A_r = Eigen::Matrix<Scalar,3,1>(A_v.X(), A_v.Y(), A_v.Z());
#else
    gazebo::math::Vector3 A_v = rotationBToA.RotateVector(gazebo::math::Vector3(B_r.x(), B_r.y(), B_r.z()));
    A_r = Eigen::Matrix<Scalar,3,1>(A_v.x, A_v.y, A_v.z);
#endif
  }


  inline static void boxPlus(Rotation& res, const Rotation& rotation, const Eigen::Vector3d& velocity) {
    // Implement res = rotation.boxPlus(vector);

    const double velocityAmplitude = velocity.norm();
    Eigen::Vector3d velocityAxis = velocity/velocityAmplitude;

    Rotation rotExp;
#if (GAZEBO_MAJOR_VERSION >= 8)
    rotExp.Axis(static_cast<double>(velocityAxis.x()),
                static_cast<double>(velocityAxis.y()),
                static_cast<double>(velocityAxis.z()),
                velocityAmplitude);
#else
    rotExp.SetFromAxis(static_cast<double>(velocityAxis.x()),
                       static_cast<double>(velocityAxis.y()),
                       static_cast<double>(velocityAxis.z()),
                       velocityAmplitude);
#endif
    res = rotExp*rotation;

  }

//  inline static void testRotation(const Rotation& expected, const Rotation& actual) {
//    // Implement EXPECT_NEAR(expected, actual, 1.0e-6);
//    EXPECT_NEAR(expected.w, actual.w, 1.0e-6);
//    EXPECT_NEAR(expected.x, actual.x, 1.0e-6);
//    EXPECT_NEAR(expected.y, actual.y, 1.0e-6);
//    EXPECT_NEAR(expected.z, actual.z, 1.0e-6);
//  }
};

}

TEST(GazeboKindr, GeometricalInterpretation)
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  kindr::ConventionTest<ignition::math::Quaterniond, Eigen::Vector3d, double>::testGeometricalInterpretation();
#else
  kindr::ConventionTest<gazebo::math::Quaternion, Eigen::Vector3d, double>::testGeometricalInterpretation();
#endif
}

TEST(GazeboKindr, Concatenation)
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  kindr::ConventionTest<ignition::math::Quaterniond, Eigen::Vector3d, double>::testConcatenation();
#else
  kindr::ConventionTest<gazebo::math::Quaternion, Eigen::Vector3d, double>::testConcatenation();
#endif
}


TEST(GazeboKindr, Rotation)
{
 #if (GAZEBO_MAJOR_VERSION >= 8)
  kindr::ConventionTest<ignition::math::Quaterniond, Eigen::Vector3d, double>::testRotationMatrix();
#else
  kindr::ConventionTest<gazebo::math::Quaternion, Eigen::Vector3d, double>::testRotationMatrix();
#endif
}

TEST(GazeboKindr, BoxPlus)
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  kindr::ConventionTest<ignition::math::Quaterniond, Eigen::Vector3d, double>::testBoxPlus();
#else
  kindr::ConventionTest<gazebo::math::Quaternion, Eigen::Vector3d, double>::testBoxPlus();
#endif
}

