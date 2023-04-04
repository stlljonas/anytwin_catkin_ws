/*!
 * @file    ik_tests.cpp
 * @author  Dario Bellicoso
 * @date    Sep, 2015
 */

// model
#include "anymal_model/AnymalModel.hpp"

// kindr tests
#include "kindr/common/gtest_eigen.hpp"

// testi fixture
#include "TestAnymalModel.hpp"

// stl
#include <utility>

using namespace anymal_model;

class InverseKintematicsTest : public TestAnymalModel {
 public:
};

TEST_F(InverseKintematicsTest, ikTest) {
  initModel("anymal_minimal");

  AnymalState state;
  state.setZero();
  getModelPtr()->setState(state, true);

  auto& model = *getModelPtr();

  constexpr auto numIterations = 1000;

  const Eigen::Vector3d randomAmplitude = Eigen::Vector3d(3.0, 2.0, 2.0);

  for (const auto limbKey : internal::limbKeys) {
    const auto limb = std::get<2>(limbKey.second);
    const auto branch = getBranchEnumFromLimbEnum(limb);

    for (auto k = 0; k < numIterations; k++) {
      Eigen::Vector3d legJointsExpected = randomAmplitude.cwiseProduct(Eigen::Vector3d::Random());

      if (std::abs(legJointsExpected(2)) < 0.0079) {
        legJointsExpected(2) = 0.008;
      }

      if (internal::mapLimbIdToLongitudinalId[limb] == LongitudinalEnum::FORE) {
        if (legJointsExpected(2) > 0.0) {
          legJointsExpected(2) *= -1.0;
        }
      } else {
        if (legJointsExpected(2) < 0.0) {
          legJointsExpected(2) *= -1.0;
        }
      }

      AnymalState::JointPositions jointPositions = state.getJointPositions();
      const auto limbId = getLimbUIntFromBranchEnum(branch);
      jointPositions.toImplementation().segment<numDofLeg>(numDofLeg * limbId) = legJointsExpected;
      state.setJointPositions(jointPositions);
      model.setState(state);

      Eigen::Vector3d position;
      model.getPositionBodyToBody(position, BranchEnum::BASE, BodyNodeEnum::BASE, branch, BodyNodeEnum::FOOT, CoordinateFrameEnum::BASE);

      Eigen::Vector3d legJointsComputed;
      model.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(legJointsComputed, position, limbId);

      std::string msg = "iteration: " + std::to_string(k) + " leg name: " + getLimbStringFromLimbEnum(limb);
      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(legJointsExpected, legJointsComputed, 1.0, msg, 1.0e-6);
    }
  }
}
