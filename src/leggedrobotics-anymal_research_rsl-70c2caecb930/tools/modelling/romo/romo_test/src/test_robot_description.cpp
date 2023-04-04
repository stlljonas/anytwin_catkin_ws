#include <romo_test/ExampleDescription.hpp>
#include <romo_test/ExampleContainersRos.hpp>

#include <gtest/gtest.h>
#include <algorithm>
#include <iostream>

#include <Eigen/Core>

TEST(RobotDescriptionRos, Convert) {
  using ECRos = romo_test::ExampleContainersRos;
  ECRos::ActuatorReadingsRos::type reading;
  reading.read();
  ECRos::ActuatorReadingsRos::msgType readingRos =
      ECRos::ActuatorReadingsRos::ConversionTrait<typename ECRos::ActuatorReadingsRos::type,
                                                  typename ECRos::ActuatorReadingsRos::msgType>::convert(reading);
  readingRos.send();
}

TEST(RobotDescription, IterateKeys) {
  using ED = romo_test::ExampleDescription;

  // auto keysItSize = std::ssize(ED::getGeneralizedVelocityKeys().begin()); // C++17
  auto keysItSize = 0U;
  auto keys = ED::getGeneralizedVelocityKeys();
  for (auto it = keys.begin(); it != keys.end(); it++) { keysItSize++; };

  ASSERT_EQ(ED::getNumDof(), keysItSize);
}

TEST(RobotDescription, MapEnums) {
  using ED = romo_test::ExampleDescription;

  ASSERT_EQ(1U, ED::getNumLimbs());

  for( const auto & actuatorKey : ED::getActuatorKeys() ) {
    auto mappedJointName = ED::mapKeyEnumToKeyName<ED::ActuatorEnum, ED::JointEnum>(actuatorKey.getEnum());
    auto expectedJointName = ED::getJointKeys().at(static_cast<ED::JointEnum>(actuatorKey.getId())).getName();
    EXPECT_STREQ(mappedJointName, expectedJointName);

    auto mappedLimbId = ED::mapKeyEnumToKeyId<ED::ActuatorEnum, ED::LimbEnum>(actuatorKey.getEnum());
    auto expectedLimbId = ED::getLimbKeys().begin()->getId(); // there is only one limb, so all should map to this
    EXPECT_EQ(mappedLimbId, expectedLimbId);
  }
}

TEST(RobotDescription, FixedSizeEigen) {
  using ED = romo_test::ExampleDescription;

  Eigen::Matrix<std::string, ED::getNumDof(), 1> myVec, myVecExpected;

  myVecExpected << "L_X", "L_Y", "L_Z", "A_X", "A_Y", "A_Z", "J1", "J2", "J3";

  for( const auto & key : ED::getGeneralizedVelocityKeys() ) {
    myVec(key.getId()) = key.getName();
  }

  ASSERT_TRUE(myVec.cwiseEqual(myVecExpected).all());
}
