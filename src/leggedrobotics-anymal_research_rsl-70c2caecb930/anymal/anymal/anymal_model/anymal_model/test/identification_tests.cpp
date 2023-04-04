/*!
 * @file     identification_tests.cpp
 * @author   Ioannis Mandralis
 * @date     Oct 2019
 */

#include <gtest/gtest.h>
#include <cmath>
#include <ctime>
#include "TestAnymalModel.hpp"
#include "kindr/common/gtest_eigen.hpp"

#ifndef ROMO_TEST_DYN_ZERO_TOL
#define ROMO_TEST_DYN_ZERO_TOL 1.0e-8
#endif

using AD = anymal_model::AD;
using AnymalState = anymal_model::AnymalState;
using JointPositions = anymal_model::JointPositions;
using JointVelocities = anymal_model::JointVelocities;
using JointAccelerations = anymal_model::JointAccelerations;

class IdentificationTests : public TestAnymalModel {
 public:
};

const Eigen::IOFormat eigenFormat(4, 0, ",", "\n", "[", "]");
bool verbose = false;

TEST_F(IdentificationTests, getBranchMoveableLinksCount) {  // NOLINT
  init();                                                   // Loads starleth model (starleth_unit_tests.urdf)
  BranchEnum branchEnum = AD::BranchEnum::LF_LEG;
  int numLinks = model_->getBranchMoveableLinksCount(branchEnum);
  if (verbose) {
    std::cout << "The number of moveable links on the LF_LEG branch are: " << numLinks << std::endl;
  }
  ASSERT_EQ(3, numLinks);
}

TEST_F(IdentificationTests, getBranchInertialParameters) {  // NOLINT
  init();                                                   // Loads starleth model (starleth_unit_tests.urdf)
  BranchEnum branchEnum = AD::BranchEnum::LF_LEG;
  Eigen::VectorXd inertialParameters = model_->getBranchInertialParameters(branchEnum);
  if (verbose) {
    std::cout << "The inertial parameters of branch LF_LEG are: " << std::endl << inertialParameters << std::endl;
  }
  ASSERT_EQ(inertialParameters(0), 1.8);
  ASSERT_EQ(inertialParameters(1), 0);
  ASSERT_EQ(inertialParameters(2), 0);
  ASSERT_EQ(inertialParameters(3), -0.099);
  ASSERT_EQ(inertialParameters(4), 0.01472);
  ASSERT_EQ(inertialParameters(5), 0);
  ASSERT_EQ(inertialParameters(6), 0);
  ASSERT_EQ(inertialParameters(7), 0.00848);
  ASSERT_EQ(inertialParameters(8), 0);
  ASSERT_EQ(inertialParameters(9), 0.00721);
  // expect all the inertias to already be in the joint frame. Compare to analytical values from huygens - steiner theorem.
}

TEST_F(IdentificationTests, getBodyInertialParameters_LF_HIP) {  // NOLINT
  init();                                                        // Loads starleth model (starleth_unit_tests.urdf)
  BodyEnum bodyEnum = AD::BodyEnum::LF_HIP;
  Eigen::VectorXd inertialParameters = model_->getBodyInertialParameters(bodyEnum);
  if (verbose) {
    std::cout << "The inertial parameters of body LF_HIP are: " << std::endl << inertialParameters << std::endl;
  }
  ASSERT_EQ(inertialParameters(0), 1.8);
  ASSERT_EQ(inertialParameters(1), 0);
  ASSERT_EQ(inertialParameters(2), 0);
  ASSERT_EQ(inertialParameters(3), -0.099);
  ASSERT_EQ(inertialParameters(4), 0.01472);
  ASSERT_EQ(inertialParameters(5), 0);
  ASSERT_EQ(inertialParameters(6), 0);
  ASSERT_EQ(inertialParameters(7), 0.00848);
  ASSERT_EQ(inertialParameters(8), 0);
  ASSERT_EQ(inertialParameters(9), 0.00721);
  // expect all the inertias to already be in the joint frame. Compare to analytical values from huygens - steiner theorem.
}

TEST_F(IdentificationTests, getInertialParameters) {  // NOLINT
  init();                                             // Loads starleth model (starleth_unit_tests.urdf)
  Eigen::VectorXd inertialParameters = model_->getInertialParameters();
  if (verbose) {
    std::cout << "The inertial parameters are: " << std::endl << inertialParameters << std::endl;
  }
  ASSERT_EQ(inertialParameters(0), 20.7);
  ASSERT_EQ(inertialParameters(1), 0.0);
  ASSERT_EQ(inertialParameters(2), 0.0);
  ASSERT_EQ(inertialParameters(3), 0.0);
  ASSERT_EQ(inertialParameters(4), 0.226);
  ASSERT_EQ(inertialParameters(5), 0.0);
  ASSERT_EQ(inertialParameters(6), 0.0);
  ASSERT_EQ(inertialParameters(7), 0.393);
  ASSERT_EQ(inertialParameters(8), 0.0);
  ASSERT_EQ(inertialParameters(9), 0.553);
  ASSERT_EQ(inertialParameters(10), 1.8);
  ASSERT_EQ(inertialParameters(11), 0);
  ASSERT_EQ(inertialParameters(12), 0);
  ASSERT_EQ(inertialParameters(13), -0.099);
  ASSERT_EQ(inertialParameters(14), 0.01472);
  ASSERT_EQ(inertialParameters(15), 0);
  ASSERT_EQ(inertialParameters(16), 0);
  ASSERT_EQ(inertialParameters(17), 0.00848);
  ASSERT_EQ(inertialParameters(18), 0);
  ASSERT_EQ(inertialParameters(19), 0.00721);
  ASSERT_EQ(inertialParameters(120), 0.32);
  // expect all the inertias to already be in the joint frame. Compare to analytical values from huygens - steiner theorem.
}

TEST_F(IdentificationTests, testGetBranchRegressor_LF_LEG) {  // NOLINT
  // Initialize the model from the URDF
  init();  // Loads starleth model (starleth_unit_tests.urdf)
  srand(time(nullptr));

  // Declare branch and get numLinks_, dofCount_ and actuatorCount_:
  using RD = anymal_model::AnymalModel::RD;
  BranchEnum branchEnum = AD::BranchEnum::LF_LEG;
  int numLinks_ = model_->getBranchMoveableLinksCount(branchEnum);  // number of moveable links per branch
  auto dofCount_ = model_->getDofCount();

  AnymalState state;
  for (int i = 0; i < 10; i++) {
    state.setZero();
    JointPositions jointPositions;

    jointPositions.setZero();
    jointPositions.toImplementation().segment(RD::getBranchStartIndexInU(branchEnum), numLinks_) = Eigen::VectorXd::Random(numLinks_);
    state.setJointPositions(jointPositions);

    if (verbose) {
      std::cout << "jointPositions are: " << std::endl << jointPositions << std::endl << std::endl;
    }

    JointVelocities jointVelocities;
    jointVelocities.setZero();
    jointVelocities(0) = rand() / (RAND_MAX + 1.);
    jointVelocities(1) = rand() / (RAND_MAX + 1.);
    jointVelocities(2) = rand() / (RAND_MAX + 1.);
    state.setJointVelocities(jointVelocities);

    if (verbose) {
      std::cout << "jointVelocities are: " << std::endl << jointPositions << std::endl << std::endl;
    }

    JointAccelerations jointAccelerations;
    jointAccelerations.setZero();
    jointAccelerations(0) = rand() / (RAND_MAX + 1.);
    jointAccelerations(1) = rand() / (RAND_MAX + 1.);
    jointAccelerations(2) = rand() / (RAND_MAX + 1.);
    state.setJointAccelerations(jointAccelerations);

    model_->setState(state, true, true, false);
    if (verbose) {
      std::cout << "The State is: " << std::endl << state << std::endl << std::endl;
    }

    // Declare the state variables and random actuation torques:
    Eigen::VectorXd q = Eigen::VectorXd::Zero(dofCount_);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(dofCount_);
    Eigen::VectorXd ddq = Eigen::VectorXd::Zero(dofCount_);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(dofCount_);

    // Set the joint positions and velocities to state values:
    q.segment(6, dofCount_ - 6) = state.getJointPositions().toImplementation();
    dq.segment(6, dofCount_ - 6) = state.getJointVelocities().toImplementation();
    ddq.segment(6, dofCount_ - 6) = state.getJointAccelerations().toImplementation();

    if (verbose) {
      std::cout << "q is: " << std::endl << q << std::endl << std::endl;
      std::cout << "dq is: " << std::endl << dq << std::endl << std::endl;
      std::cout << "ddq is: " << std::endl << ddq << std::endl << std::endl;
    }

    // Update RBDL kinematics taking into account the accelerations.
    RigidBodyDynamics::UpdateKinematics(model_->getRbdlModel(), q, dq, ddq, true);

    if (verbose) {
      std::cout << "The Joint Accelerations are: " << std::endl << ddq << std::endl << std::endl;
    }

    // compute the torques required to obtain desired ddq:
    Eigen::MatrixXd baseJacobian = Eigen::MatrixXd::Zero(6, dofCount_);
    model_->getJacobianSpatialWorldToBody(baseJacobian, AD::BranchEnum::BASE, AD::BodyNodeEnum::BASE, AD::CoordinateFrameEnum::WORLD);
    if (verbose) {
      std::cout << "The Base Jacobian is: " << std::endl << baseJacobian.format(eigenFormat) << std::endl << std::endl;
    }

    // Computing QR-Decomposition of transpose of base Jacobian:
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dofCount_, dofCount_);

    Eigen::FullPivHouseholderQR<Eigen::MatrixXd> QR(baseJacobian.transpose());
    Q = QR.matrixQ();
    int rankBaseJacobian = QR.rank();

    // Define the Su matrix: (baseJacobian is rank 6)
    Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(dofCount_ - rankBaseJacobian, dofCount_);
    Su.block(0, rankBaseJacobian, dofCount_ - rankBaseJacobian, dofCount_ - rankBaseJacobian) =
        Eigen::MatrixXd::Identity(dofCount_ - rankBaseJacobian, dofCount_ - rankBaseJacobian);

    auto projDummy = Su * Q.transpose() * (model_->getActuatorSelectionMatrix()).transpose();
    auto pinv = projDummy.completeOrthogonalDecomposition().pseudoInverse();
    auto proj = pinv * Su * Q.transpose();

    if (verbose) {
      std::cout << "The Q-matrix of the transpose jacobian is: " << std::endl << Q.format(eigenFormat) << std::endl;
      std::cout << "The rank of the base jacobian is: " << rankBaseJacobian << std::endl << std::endl;
      std::cout << "The Projection Matrix is: " << std::endl << proj << std::endl << std::endl;
      std::cout << "The Mass Matrix is: " << std::endl << model_->getMassInertiaMatrix() << std::endl << std::endl;
      std::cout << "The Nonlinear Effects are: " << std::endl << model_->getNonlinearEffects() << std::endl << std::endl;
    }

    tau = proj * (model_->getMassInertiaMatrix() * ddq + model_->getNonlinearEffects());

    if (verbose) {
      std::cout << "The Actuation Torques are: " << std::endl << tau << std::endl << std::endl;
    }

    // Declare and compute the regressor:
    Eigen::MatrixXd regressor = model_->getBranchRegressor(branchEnum, verbose);
    if (verbose) {
      std::cout << "The Regressor is: " << std::endl << regressor.format(eigenFormat) << std::endl << std::endl;
    }

    // Get the Inertial Parameters (with inertias already referred to joint origin frame):
    Eigen::VectorXd inertialParameters = model_->getBranchInertialParameters(branchEnum);
    if (verbose) {
      std::cout << "The Inertial Parameters are: " << std::endl << inertialParameters.format(eigenFormat) << std::endl << std::endl;
      std::cout << "The number of Inertial Parameters is: " << std::endl << inertialParameters.size() << std::endl << std::endl;
    }

    // Obtain the torque using the regressor (external forces are zero normally)
    Eigen::VectorXd tauRegressor = Eigen::VectorXd::Zero(dofCount_);
    tauRegressor = regressor * inertialParameters;
    if (verbose) {
      std::cout << "tauRegressor is: " << std::endl << tauRegressor.format(eigenFormat) << std::endl << std::endl;
    }

    // Print the commanded torque for verification:
    if (verbose) {
      std::cout << "tauModel is: " << std::endl << tau.format(eigenFormat) << std::endl << std::endl;
    }

    // Compare the two torque computations at 1% tolerance:
    std::string msg;
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(tauRegressor, tau.segment<3>(0), 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);
  }
}

TEST_F(IdentificationTests, testGetBranchRegressorAllLegs) {  // NOLINT
  // Initialize the model from the URDF
  init();  // Loads starleth model (starleth_unit_tests.urdf)
  srand(time(nullptr));

  // Declare robot description and get numLinks_, dofCount_ and actuatorCount_:
  // int numLinks_ = model_->getBranchMoveableLinksCount(AD::BranchEnum::LF_LEG);  // number of moveable links per branch
  auto dofCount_ = model_->getDofCount();

  AnymalState state;
  for (int i = 0; i < 10; i++) {
    state.setZero();
    JointPositions jointPositions;

    jointPositions.setRandom();
    state.setJointPositions(jointPositions);

    if (verbose) {
      std::cout << "jointPositions are: " << std::endl << jointPositions << std::endl << std::endl;
    }

    JointVelocities jointVelocities;
    jointVelocities.setRandom();
    state.setJointVelocities(jointVelocities);

    if (verbose) {
      std::cout << "jointVelocities are: " << std::endl << jointPositions << std::endl << std::endl;
    }

    JointAccelerations jointAccelerations;
    jointAccelerations.setRandom();
    state.setJointAccelerations(jointAccelerations);

    model_->setState(state, true, true, false);
    if (verbose) {
      std::cout << "The State is: " << std::endl << state << std::endl << std::endl;
    }

    // Declare the state variables and random actuation torques:
    Eigen::VectorXd q = Eigen::VectorXd::Zero(dofCount_);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(dofCount_);
    Eigen::VectorXd ddq = Eigen::VectorXd::Zero(dofCount_);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(dofCount_);

    // Set the joint positions and velocities to state values:
    q.segment(6, 12) = state.getJointPositions().toImplementation();
    dq.segment(6, 12) = state.getJointVelocities().toImplementation();
    ddq.segment(6, 12) = state.getJointAccelerations().toImplementation();

    if (verbose) {
      std::cout << "q is: " << std::endl << q << std::endl << std::endl;
      std::cout << "dq is: " << std::endl << dq << std::endl << std::endl;
      std::cout << "ddq is: " << std::endl << ddq << std::endl << std::endl;
    }

    // Update RBDL kinematics taking into account the accelerations.
    RigidBodyDynamics::UpdateKinematics(model_->getRbdlModel(), q, dq, ddq, true);

    if (verbose) {
      std::cout << "The Joint Accelerations are: " << std::endl << ddq << std::endl << std::endl;
    }

    // compute the torques required to obtain desired ddq:
    Eigen::MatrixXd baseJacobian = Eigen::MatrixXd::Zero(6, dofCount_);
    model_->getJacobianSpatialWorldToBody(baseJacobian, AD::BranchEnum::BASE, AD::BodyNodeEnum::BASE, AD::CoordinateFrameEnum::WORLD);
    if (verbose) {
      std::cout << "The Base Jacobian is: " << std::endl << baseJacobian.format(eigenFormat) << std::endl << std::endl;
    }

    // Computing QR-Decomposition of transpose of base Jacobian:
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dofCount_, dofCount_);

    Eigen::FullPivHouseholderQR<Eigen::MatrixXd> QR(baseJacobian.transpose());
    Q = QR.matrixQ();
    int rankBaseJacobian = QR.rank();

    if (verbose) {
      std::cout << "The Q-matrix of the transpose jacobian is: " << std::endl << Q.format(eigenFormat) << std::endl;
      std::cout << "The rank of the base jacobian is: " << rankBaseJacobian << std::endl << std::endl;
    }

    // Define the Su matrix: (baseJacobian is rank 6)
    Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(dofCount_ - rankBaseJacobian, dofCount_);
    Su.block(0, rankBaseJacobian, dofCount_ - rankBaseJacobian, dofCount_ - rankBaseJacobian) =
        Eigen::MatrixXd::Identity(dofCount_ - rankBaseJacobian, dofCount_ - rankBaseJacobian);

    auto projDummy = Su * Q.transpose() * (model_->getActuatorSelectionMatrix()).transpose();
    auto pinv = projDummy.completeOrthogonalDecomposition().pseudoInverse();
    auto proj = pinv * Su * Q.transpose();  // DEBUG AGAINNNNNN

    if (verbose) {
      std::cout << "The Projection Matrix is: " << std::endl << proj << std::endl << std::endl;
    }

    if (verbose) {
      std::cout << "The Mass Matrix is: " << std::endl << model_->getMassInertiaMatrix() << std::endl << std::endl;
      std::cout << "The Nonlinear Effects are: " << std::endl << model_->getNonlinearEffects() << std::endl << std::endl;
    }

    tau = proj * (model_->getMassInertiaMatrix() * ddq + model_->getNonlinearEffects());

    if (verbose) {
      std::cout << "The Actuation Torques are: " << std::endl << tau << std::endl << std::endl;
    }

    // Compute the regressor:
    Eigen::MatrixXd regressorLF = model_->getBranchRegressor(AD::BranchEnum::LF_LEG, verbose);
    Eigen::MatrixXd regressorRF = model_->getBranchRegressor(AD::BranchEnum::RF_LEG, verbose);
    Eigen::MatrixXd regressorLH = model_->getBranchRegressor(AD::BranchEnum::LH_LEG, verbose);
    Eigen::MatrixXd regressorRH = model_->getBranchRegressor(AD::BranchEnum::RH_LEG, verbose);

    if (verbose) {
      std::cout << "The RegressorLF is: " << std::endl << regressorLF.format(eigenFormat) << std::endl << std::endl;
      std::cout << "The RegressorRF is: " << std::endl << regressorRF.format(eigenFormat) << std::endl << std::endl;
      std::cout << "The RegressorLH is: " << std::endl << regressorLH.format(eigenFormat) << std::endl << std::endl;
      std::cout << "The RegressorRH is: " << std::endl << regressorRH.format(eigenFormat) << std::endl << std::endl;
    }

    // Get the Inertial Parameters (with inertias already referred to joint origin frame):
    Eigen::VectorXd inertialParametersLF = model_->getBranchInertialParameters(AD::BranchEnum::LF_LEG);
    Eigen::VectorXd inertialParametersRF = model_->getBranchInertialParameters(AD::BranchEnum::RF_LEG);
    Eigen::VectorXd inertialParametersLH = model_->getBranchInertialParameters(AD::BranchEnum::LH_LEG);
    Eigen::VectorXd inertialParametersRH = model_->getBranchInertialParameters(AD::BranchEnum::RH_LEG);

    if (verbose) {
      std::cout << "The Inertial Parameters LF are: " << std::endl << inertialParametersLF.format(eigenFormat) << std::endl << std::endl;
      std::cout << "The Inertial Parameters RF are: " << std::endl << inertialParametersRF.format(eigenFormat) << std::endl << std::endl;
      std::cout << "The Inertial Parameters LH are: " << std::endl << inertialParametersLH.format(eigenFormat) << std::endl << std::endl;
      std::cout << "The Inertial Parameters RH are: " << std::endl << inertialParametersRH.format(eigenFormat) << std::endl << std::endl;
      std::cout << "The number of Inertial Parameters per leg is: " << std::endl << inertialParametersLF.size() << std::endl << std::endl;
    }

    // Obtain the torque using the regressor (external forces are zero normally)
    Eigen::VectorXd tauRegressorLF = Eigen::VectorXd::Zero(dofCount_);
    Eigen::VectorXd tauRegressorRF = Eigen::VectorXd::Zero(dofCount_);
    Eigen::VectorXd tauRegressorLH = Eigen::VectorXd::Zero(dofCount_);
    Eigen::VectorXd tauRegressorRH = Eigen::VectorXd::Zero(dofCount_);

    tauRegressorLF = regressorLF * inertialParametersLF;
    tauRegressorRF = regressorRF * inertialParametersRF;
    tauRegressorLH = regressorLH * inertialParametersLH;
    tauRegressorRH = regressorRH * inertialParametersRH;

    if (verbose) {
      std::cout << "tauRegressorLF is: " << std::endl << tauRegressorLF.format(eigenFormat) << std::endl << std::endl;
      std::cout << "tauRegressorRF is: " << std::endl << tauRegressorRF.format(eigenFormat) << std::endl << std::endl;
      std::cout << "tauRegressorLH is: " << std::endl << tauRegressorLH.format(eigenFormat) << std::endl << std::endl;
      std::cout << "tauRegressorRH is: " << std::endl << tauRegressorRH.format(eigenFormat) << std::endl << std::endl;
    }  // Print the commanded torque for verification.

    if (verbose) {
      std::cout << "tauModel is: " << std::endl << tau.format(eigenFormat) << std::endl << std::endl;
    }

    // Compare the two torque computations at 1% tolerance:
    std::string msg;
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(tauRegressorLF, tau.segment<3>(0), 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(tauRegressorRF, tau.segment<3>(3), 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(tauRegressorLH, tau.segment<3>(6), 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(tauRegressorRH, tau.segment<3>(9), 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);
  }
}
