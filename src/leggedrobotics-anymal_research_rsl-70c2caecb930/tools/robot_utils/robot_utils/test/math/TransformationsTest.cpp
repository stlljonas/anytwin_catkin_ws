/*!
 * @file     TransformationsTest.cpp
 * @author   Christian Gehring
 * @date     May 22, 2016
 * @brief
 */

#include <gtest/gtest.h>
#include <kindr/Core>
#include <kindr/common/gtest_eigen.hpp>

#include "robot_utils/math/Transformations.hpp"

TEST(TransformationsTest, getPerspectiveTransform_identity) {  // NOLINT
  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  src.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  dst.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));

  src.emplace_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  dst.emplace_back(Eigen::Vector3d(0.0, 1.0, 1.0));

  src.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  dst.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));

  src.emplace_back(Eigen::Vector3d(1.0, 0.0, 1.0));
  dst.emplace_back(Eigen::Vector3d(1.0, 0.0, 1.0));

  Eigen::Matrix3d M = robot_utils::getPerspectiveTransform(src, dst);

  std::vector<Eigen::Vector3d> calcDest(4);
  for (int i = 0; i < 4; i++) {
    calcDest[i] = M * src[i];
    calcDest[i] /= calcDest[i].z();
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "identity");
  }
}

TEST(TransformationsTest, getPerspectiveTransform_translation) {  // NOLINT
  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  src.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.emplace_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  src.emplace_back(Eigen::Vector3d(1.0, 0.0, 1.0));

  for (int i = 0; i < 4; i++) {
    dst.emplace_back(src[i] + Eigen::Vector3d(0.1, 0.2, 0.0));
  }

  Eigen::Matrix3d M = robot_utils::getPerspectiveTransform(src, dst);

  std::vector<Eigen::Vector3d> calcDest(4);
  for (int i = 0; i < 4; i++) {
    calcDest[i] = M * src[i];
    calcDest[i] /= calcDest[i].z();
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "translation");
  }
}

TEST(TransformationsTest, getPerspectiveTransform_rotation) {  // NOLINT
  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  kindr::EulerAnglesZyxPD rot(10.0 / 180.0 * M_PI, 20.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI);

  src.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.emplace_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  src.emplace_back(Eigen::Vector3d(1.0, 0.0, 1.0));

  for (int i = 0; i < 4; i++) {
    dst.emplace_back(rot.rotate(src[i]));
    dst.back().z() = 1.0;
  }

  Eigen::Matrix3d M = robot_utils::getPerspectiveTransform(src, dst);

  std::vector<Eigen::Vector3d> calcDest(4);
  for (int i = 0; i < 4; i++) {
    calcDest[i] = M * src[i];
    calcDest[i] /= calcDest[i].z();
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "rotation");
  }
}

TEST(TransformationsTest, getPerspectiveTransform_scale) {  // NOLINT
  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  kindr::EulerAnglesZyxPD rot(10.0 / 180.0 * M_PI, 20.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI);

  src.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.emplace_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  src.emplace_back(Eigen::Vector3d(1.0, 0.0, 1.0));

  Eigen::Matrix3d scale = Eigen::Matrix3d::Zero();
  scale(0, 0) = 0.1;
  scale(1, 1) = 0.2;

  for (int i = 0; i < 4; i++) {
    dst.emplace_back(scale * src[i]);
    dst.back().z() = 1.0;
  }

  Eigen::Matrix3d M = robot_utils::getPerspectiveTransform(src, dst);

  std::vector<Eigen::Vector3d> calcDest(4);
  for (int i = 0; i < 4; i++) {
    calcDest[i] = M * src[i];
    calcDest[i] /= calcDest[i].z();
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "scale");
  }
}

TEST(TransformationsTest, getAffineTransform_identity) {  // NOLINT
  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  src.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  dst.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));

  src.emplace_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  dst.emplace_back(Eigen::Vector3d(0.0, 1.0, 1.0));

  src.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  dst.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));

  Eigen::Matrix3d M = robot_utils::getAffineTransform(src, dst);

  std::vector<Eigen::Vector3d> calcDest(3);
  for (int i = 0; i < 3; i++) {
    calcDest[i] = M * src[i];
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "identity");
  }
}

TEST(TransformationsTest, getAffineTransform_translation) {  // NOLINT
  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  src.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.emplace_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));

  for (int i = 0; i < 3; i++) {
    dst.emplace_back(src[i] + Eigen::Vector3d(0.1, 0.2, 0.0));
  }

  Eigen::Matrix3d M = robot_utils::getAffineTransform(src, dst);

  std::vector<Eigen::Vector3d> calcDest(3);
  for (int i = 0; i < 3; i++) {
    calcDest[i] = M * src[i];
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "translation");
  }
}

TEST(TransformationsTest, getAffineTransform_rotation) {  // NOLINT
  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  kindr::EulerAnglesZyxPD rot(10.0 / 180.0 * M_PI, 20.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI);

  src.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.emplace_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));

  for (int i = 0; i < 3; i++) {
    dst.emplace_back(rot.rotate(src[i]));
    dst.back().z() = 1.0;
  }

  Eigen::Matrix3d M = robot_utils::getAffineTransform(src, dst);

  std::vector<Eigen::Vector3d> calcDest(3);
  for (int i = 0; i < 3; i++) {
    calcDest[i] = M * src[i];
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "rotation");
  }
}

TEST(TransformationsTest, getAffineTransform_scale) {  // NOLINT
  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  kindr::EulerAnglesZyxPD rot(10.0 / 180.0 * M_PI, 20.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI);

  src.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.emplace_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));

  Eigen::Matrix3d scale = Eigen::Matrix3d::Zero();
  scale(0, 0) = 0.1;
  scale(1, 1) = 0.2;

  for (int i = 0; i < 3; i++) {
    dst.emplace_back(scale * src[i]);
    dst.back().z() = 1.0;
  }

  Eigen::Matrix3d M = robot_utils::getAffineTransform(src, dst);

  std::vector<Eigen::Vector3d> calcDest(3);
  for (int i = 0; i < 3; i++) {
    calcDest[i] = M * src[i];
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "scale");
  }
}
