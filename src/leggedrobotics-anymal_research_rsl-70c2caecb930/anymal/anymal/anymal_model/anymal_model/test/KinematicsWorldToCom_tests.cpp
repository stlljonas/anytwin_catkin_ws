/*!
 * @file    KinematicsWorldToCom_tests.cpp
 * @author  Christian Gehring, Dario Bellicoso
 * @date    Sep, 2015
 */

#include <gtest/gtest.h>
#include <chrono>
#include "TestAnymalModel.hpp"
#include "gtest_anymal_model.hpp"
#include "kindr/common/gtest_eigen.hpp"

using AnymalModel = anymal_model::AnymalModel;

class KinematicsWorldToComTest : public TestAnymalModel {
 public:
};

Eigen::Vector3d getLinearVelocityComByJacobian(AnymalModel& anymalModel, const CoordinateFrameEnum& frame) {
  using namespace anymal_model;
  Eigen::MatrixXd spatialJacobianCom = Eigen::MatrixXd::Zero(6, anymalModel.getDofCount());
  Eigen::Vector3d linearVelocity;

  for (const auto& body : anymalModel.getBodyContainer()) {
    if (!anymalModel.getRbdlModel().IsFixedBodyId(body.getRbdlBodyId())) {
      Eigen::MatrixXd spatialJacobianBody = Eigen::MatrixXd::Zero(6, anymalModel.getDofCount());

      RigidBodyDynamics::CalcSpatialJacobianWorldToPointInWorldFrame(
          anymalModel.getRbdlModel(), anymalModel.getStateGeneralizedPositionsQuaternionRBDL(), body.getRbdlBodyId(),
          body.getPositionBodyToBodyCom(CoordinateFrameEnum::BODY), spatialJacobianBody, false);
      spatialJacobianCom += body.getMass() * spatialJacobianBody;
    }
  }

  linearVelocity = (spatialJacobianCom.bottomRows(3) * anymalModel.getStateGeneralizedVelocitiesAngularRBDL()) / anymalModel.getTotalMass();
  switch (frame) {
    case (CoordinateFrameEnum::WORLD):
      break;
    case (CoordinateFrameEnum::BASE): {
      const Eigen::Matrix3d& orientationWorldToBase = anymalModel.getOrientationWorldToBody(BodyEnum::BASE);
      linearVelocity = orientationWorldToBase * (linearVelocity);
    } break;
    default:
      throw std::runtime_error("[AnymalModel::getLinearVelocityCom] frame is not supported!");
      break;
  }

  return linearVelocity;
}

TEST_F(KinematicsWorldToComTest, getLinearVelocityComInWorldFrame) {
  initModel("starleth_unit_test", USE_QUATERNION);

  AnymalState state;
  state.setRandom();
  getModelPtr()->setState(state, true, true);

  std::chrono::time_point<std::chrono::steady_clock> start, end;
  start = std::chrono::steady_clock::now();
  //-- Compute expected linear velocity by Jacobian
  Eigen::Vector3d expectedVelocity = getLinearVelocityComByJacobian(*getModelPtr(), CoordinateFrameEnum::WORLD);
  //--
  end = std::chrono::steady_clock::now();
  //  std::cout << "Time: Jacobian method: (nsec): " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() <<
  //  std::endl;

  start = std::chrono::steady_clock::now();
  Eigen::Vector3d computedVelocity = getModelPtr()->getLinearVelocityCom(CoordinateFrameEnum::WORLD);
  end = std::chrono::steady_clock::now();
  //  std::cout << "Time: RBDL (nsec): " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << std::endl;

  std::string msg = "getLinearVelocityComInWorldFrame";
  KINDR_ASSERT_DOUBLE_MX_EQ(expectedVelocity, computedVelocity, 1.0, msg);
}
