/*!
 * @file    Dynamics_tests.cpp
 * @author  Dario Bellicoso
 * @date    Sep, 2015
 */

// Starleth dynamics
#include "starleth_dynamics/StarlethDynamics.hpp"

// gtest
#include <gtest/gtest.h>
#include "gtest_anymal_model.hpp"

// test anymal model
#include "TestAnymalModel.hpp"

using namespace anymal_model;
const Eigen::IOFormat eigenFormat(2, 0, ",", "\n", "[", "]");

class DynamicsTest : public TestAnymalModel {
 public:
};

TEST_F(DynamicsTest, getGravityTerms) {
  initModel("starleth_unit_test");
  setRandomStateToModel();

  const GeneralizedCoordinates generalizedPositions = getStatePtr()->getGeneralizedCoordinates();
  const EulerAnglesXyz eulerXyz(getStatePtr()->getOrientationBaseToWorld());

  // Write projection matrix B which maps generalized velocities using angular velocity B_w_IB to dEuler XYZ
  Eigen::MatrixXd B = Eigen::MatrixXd::Identity(numGenCoordWholeBody, numGenCoordWholeBody);

  Eigen::MatrixXd E = eulerXyz.getMappingFromLocalAngularVelocityToDiff();

  Eigen::MatrixXd C_IB = RotationMatrix(getStatePtr()->getOrientationBaseToWorld()).matrix();
  B.block<3, 3>(3, 3) = E;

  // Get the generalized positions using Euler Xyz as parametrization.
  Eigen::VectorXd generalizedPositionsEulerAnglesXyz(numGenCoordWholeBody);
  generalizedPositionsEulerAnglesXyz.segment<3>(0) = generalizedPositions.segment<3>(0);
  generalizedPositionsEulerAnglesXyz.segment<3>(3) = eulerXyz.toImplementation();
  generalizedPositionsEulerAnglesXyz.segment<12>(6) = getStatePtr()->getJointPositions().toImplementation();

  // Compute gravity generalized forces from the Anymal Model.
  Eigen::VectorXd gravityFromAnymalModel = Eigen::VectorXd::Zero(numGenCoordWholeBody);
  getModelPtr()->getGravityTerms(gravityFromAnymalModel);

  // Compute gravity generalized forces from Proneu.
  const Eigen::VectorXd gravityFromProneu = starleth_dynamics::getGravityGeneralizedForces(generalizedPositionsEulerAnglesXyz);
  const Eigen::VectorXd gravityFromProneuProjected = B.transpose() * gravityFromProneu;

  std::string msg = "";
  ASSERT_TRUE(gravityFromProneuProjected.isApprox(gravityFromAnymalModel));
}

TEST_F(DynamicsTest, getMassMatrix) {
  initModel("starleth_unit_test");
  setRandomStateToModel();

  const GeneralizedCoordinates generalizedPositions = getStatePtr()->getGeneralizedCoordinates();
  const EulerAnglesXyz eulerXyz(getStatePtr()->getOrientationBaseToWorld());

  // Write projection matrix B which maps generalized velocities using angular velocity B_w_IB to dEuler XYZ
  Eigen::MatrixXd B = Eigen::MatrixXd::Identity(numGenCoordWholeBody, numGenCoordWholeBody);
  Eigen::MatrixXd E = eulerXyz.getMappingFromLocalAngularVelocityToDiff();

  Eigen::MatrixXd C_IB = RotationMatrix(getStatePtr()->getOrientationBaseToWorld()).matrix();
  B.block<3, 3>(3, 3) = E;

  // Get the generalized positions using Euler Xyz as parametrization.
  Eigen::VectorXd generalizedPositionsEulerAnglesXyz(numGenCoordWholeBody);
  generalizedPositionsEulerAnglesXyz.segment<3>(0) = generalizedPositions.segment<3>(0);
  generalizedPositionsEulerAnglesXyz.segment<3>(3) = eulerXyz.toImplementation();
  generalizedPositionsEulerAnglesXyz.segment<12>(6) = getStatePtr()->getJointPositions().toImplementation();

  // Compute mass matrix from anymal model. Compute it from Proneu and project it to the same space as anymal model.
  Eigen::MatrixXd massMatrixFromAnymalModel = getModelPtr()->getMassInertiaMatrix();
  Eigen::MatrixXd massMatrixFromProneuProjected = B.transpose() * starleth_dynamics::getMassMatrix(generalizedPositionsEulerAnglesXyz) * B;

  std::string msg = "";
  ASSERT_TRUE(massMatrixFromProneuProjected.isApprox(massMatrixFromAnymalModel));
}

// TEST_F(DynamicsTest, getNonlinearEffects) {
//  init();
//
//  //--- Set a random state
//  AnymalState state;
////  getStatePtr()->setJointPositions(JointPositions(Eigen::Matrix<double, 12, 1>::Random()));
////  getStatePtr()->setJointVelocities(JointVelocities(Eigen::Matrix<double, 12, 1>::Random()));
//  getStatePtr()->setRandom();
//  getStatePtr()->setAngularVelocityBaseInBaseFrame(LocalAngularVelocity());
//  getStatePtr()->setOrientationBaseToWorld(RotationQuaternion());
//
//  getModelPtr()->setState(state, true, true);
//
//  const GeneralizedCoordinates generalizedPositions = getStatePtr()->getGeneralizedCoordinates();
//  const EulerAnglesXyz eulerXyz(getStatePtr()->getOrientationBaseToWorld());
//  const EulerAnglesXyzDiff
//  dEulerXyz(eulerXyz.getMappingFromLocalAngularVelocityToDiff().matrix()*getStatePtr()->getAngularVelocityBaseInBaseFrame().toImplementation());
//
//  // Write projection matrix B which maps generalized velocities using angular velocity B_w_IB to dEuler XYZ
//  Eigen::MatrixXd B = Eigen::MatrixXd::Identity(numGenCoordWholeBody, numGenCoordWholeBody);
//  B.block<3,3>(3,3) = eulerXyz.getMappingFromLocalAngularVelocityToDiff();
//
//  Eigen::MatrixXd dB = Eigen::MatrixXd::Identity(numGenCoordWholeBody, numGenCoordWholeBody);
//  dB.block<3,3>(3,3) = dEulerXyz.getMappingFromLocalAngularVelocityToSecondDiff(eulerXyz);
//
//  // Get the generalized positions using Euler Xyz as parametrization.
//  Eigen::VectorXd generalizedPositionsEulerAnglesXyz(numGenCoordWholeBody);
//  generalizedPositionsEulerAnglesXyz.segment<3>(0) = generalizedPositions.segment<3>(0);
//  generalizedPositionsEulerAnglesXyz.segment<3>(3) = eulerXyz.toImplementation();
//  generalizedPositionsEulerAnglesXyz.segment<12>(6) = getStatePtr()->getJointPositions().toImplementation();
//
//  // Get the generalized velocities using Euler Xyz as parametrization.
//  const Eigen::VectorXd generalizedVelocitiesEulerAnglesXyz = B*getStatePtr()->getGeneralizedVelocities();
//
//  // Compute gravity generalized forces from the Anymal Model.
//  Eigen::VectorXd nonlinearEffectsFromAnymalModel = Eigen::VectorXd::Zero(numGenCoordWholeBody);
//  getModelPtr()->getNonlinearEffects(nonlinearEffectsFromAnymalModel);
//
//  // Compute gravity generalized forces from Proneu.
//  const Eigen::VectorXd nonlinearEffectsFromProneu = starleth_dynamics::getCoriolisGeneralizedForces(generalizedPositionsEulerAnglesXyz,
//                                                                                                     generalizedVelocitiesEulerAnglesXyz)
//                                                   + starleth_dynamics::getGravityGeneralizedForces(generalizedPositionsEulerAnglesXyz);
//  const Eigen::VectorXd nonlinearEffectsFromProneuProjected = B.transpose()*(nonlinearEffectsFromProneu
//      - starleth_dynamics::getMassMatrix(generalizedPositionsEulerAnglesXyz)*dB*getStatePtr()->getGeneralizedVelocities());
//
//  std::cout << "c rbdl: " << std::endl << nonlinearEffectsFromgetModelPtr()->transpose().format(eigenFormat) << std::endl;
//  std::cout << "c proneu projected: " << std::endl << nonlinearEffectsFromProneuProjected.transpose().format(eigenFormat) << std::endl;
//  std::cout << "c proneu: " << std::endl << nonlinearEffectsFromProneu.transpose().format(eigenFormat) << std::endl;
//
//  std::string msg  = "";
//  ASSERT_TRUE(nonlinearEffectsFromProneuProjected.isApprox(nonlinearEffectsFromAnymalModel));
//}
