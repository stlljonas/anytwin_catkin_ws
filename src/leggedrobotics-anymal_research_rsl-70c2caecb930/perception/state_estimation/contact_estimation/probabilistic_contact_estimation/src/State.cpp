/*
 * State.cpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include <probabilistic_contact_estimation/State.hpp>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace contact_estimation {

State::State(anymal_model::AnymalModel* model) :
    model_(model),
    numDofLimb_(3u),
    numLegs_(static_cast<unsigned int>(anymal_description::AnymalTopology::FootEnum::SIZE)),
    computeBaseAcceleration_(true),
    useWholeBodyContactForces_(false),
    massMatrix_(Eigen::MatrixXd::Zero(model_->getDofCount(), model_->getDofCount())),
    nonlinearEffects_(Eigen::VectorXd::Zero(model_->getDofCount())),
    legMassMatrixInverted_(Eigen::MatrixXd::Zero(3u, 3u)),
    selectionMatrixTransposed_(model_->getActuatorSelectionMatrix().transpose()),
    nonlinearityAndTorque_(Eigen::VectorXd::Zero(model_->getDofCount())),
    previousContactState_(ContactState::CLOSED),
    positionWorldToEndEffectorInWorldFrameVector_(Eigen::Vector3d::Zero()),
    positionWorldToEndEffectorTouchDownInWorldFrameVector_(Eigen::Vector3d::Zero()),
    generalizedVelocities_(),
    generalizedAccelerations_(),
    jointTorques_(),
    linearVelocityEndEffectorInWorldFrameVector_(),
    contactForceInWorldFrameVector_(),
    expectedGroundHeightInWorldFrame_(),
    groundHeightFilterConstant_(0.025),
    orientationWorldToPlane_(),
    filteredTerrainNormal_(),
    filteredPositionOnPlane_(),
    terrainFilterConstant_(0.05),
    jacobianTranslationWorldToEndEffectorInWorldFrameVector_(AD::JacobianTranslation::Zero()),
    jacobianTranslationTimeDerivativeWorldToEndEffectorInWorldFrameVector_(AD::JacobianTranslation::Zero())
{
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto limbEnum = AD::mapKeyEnumToKeyEnum<AD::ContactEnum, AD::LimbEnum>(contactEnum);
    numDofLimb_[contactEnum] = AD::getNumDofLimb(limbEnum);
    legMassMatrixInverted_[contactEnum].setZero(numDofLimb_[contactEnum], numDofLimb_[contactEnum]);
    jacobianTranslationWorldToEndEffectorInWorldFrameVector_[contactEnum] = AD::JacobianTranslation::Zero();
    jacobianTranslationTimeDerivativeWorldToEndEffectorInWorldFrameVector_[contactEnum] = AD::JacobianTranslation::Zero();
  }

}

bool State::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle contactEstimationHandle = handle;
  if(!tinyxml_tools::getChildHandle(contactEstimationHandle, handle, "ProbabilisticContactEstimation")) { return false; }

  TiXmlHandle terrainHandle = handle;
  if(!tinyxml_tools::getChildHandle(terrainHandle, contactEstimationHandle, "TerrainEstimation")) { return false; }

  TiXmlHandle filtereHandle = handle;
  if(!tinyxml_tools::getChildHandle(filtereHandle, terrainHandle, "Filters")) { return false; }
  if(!tinyxml_tools::loadParameter(terrainFilterConstant_, filtereHandle, "terrain_filter_constant")) { return false; }
  if(!tinyxml_tools::loadParameter(groundHeightFilterConstant_, filtereHandle, "ground_height_filter_constant")) { return false; }

  return true;
}

bool State::initialize(double dt) {
  // Initialize Filters.
  filteredTerrainNormal_.setFilterParameters(dt, terrainFilterConstant_, 1.0, Eigen::Vector3d::UnitZ());
  filteredPositionOnPlane_.setFilterParameters(dt, terrainFilterConstant_, 1.0, Eigen::Vector3d::UnitZ());

  // Previous contact locations.
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto branchEnum = AD::mapKeyEnumToKeyEnum<AD::ContactEnum, AD::BranchEnum>(contactEnum);

    model_->getPositionWorldToBody(
        positionWorldToEndEffectorTouchDownInWorldFrameVector_[contactEnum],
        branchEnum,
        AD::BodyNodeEnum::FOOT,
        AD::CoordinateFrameEnum::WORLD);

    orientationWorldToPlane_.setIdentity();
    expectedGroundHeightInWorldFrame_[contactEnum].setFilterParameters(dt, groundHeightFilterConstant_, 1.0, orientationWorldToPlane_.rotate(positionWorldToEndEffectorTouchDownInWorldFrameVector_[contactEnum] ).z());
  }

  // Initialize states.
  std::fill(previousContactState_.begin(), previousContactState_.end(), ContactState::CLOSED);
  if(!advance(dt, previousContactState_)) {
    MELO_WARN_STREAM("Failed to initialize states at iteration.");
    return false;
  }

  return true;
}

bool State::advance(double dt, const std_utils::EnumArray<AD::ContactEnum, ContactState>& contactState) {
  constexpr auto numDofBase = 6u;
  const auto baseIdInU = AD::getBranchStartIndexInU(AD::BranchEnum::BASE);

  /***************************
   * Generalized coordinates *
   ***************************/
  // Compute derivative of generalized velocities.
  if (computeBaseAcceleration_) {
    const auto generalizedVelocitiesTimeDerivative = (model_->getState().getGeneralizedVelocities()-generalizedVelocities_)/dt;
    generalizedAccelerations_.segment<AD::getNumSpatialDof()>(baseIdInU) = generalizedVelocitiesTimeDerivative.segment<AD::getNumSpatialDof()>(baseIdInU);
  } else {
    generalizedAccelerations_(0) = model_->getState().getLinearAccelerationBaseInWorldFrame().x();
    generalizedAccelerations_(1) = model_->getState().getLinearAccelerationBaseInWorldFrame().y();
    generalizedAccelerations_(2) = model_->getState().getLinearAccelerationBaseInWorldFrame().z();
    generalizedAccelerations_(3) = model_->getState().getAngularAccelerationBaseInBaseFrame().x();
    generalizedAccelerations_(4) = model_->getState().getAngularAccelerationBaseInBaseFrame().y();
    generalizedAccelerations_(5) = model_->getState().getAngularAccelerationBaseInBaseFrame().z();
  }
  generalizedAccelerations_.bottomRows(AD::getJointsDimension()) = model_->getState().getJointAccelerations().toImplementation();

  // Compute filtered generalized velocities.
  generalizedVelocities_ = model_->getState().getGeneralizedVelocities();

  // Get robot dynamics.
  massMatrix_.resize(AD::getNumDof(), AD::getNumDof());
  model_->getMassInertiaMatrix(massMatrix_);

  nonlinearEffects_.resize(AD::getNumDof());
  model_->getNonlinearEffects(nonlinearEffects_);

  // Nonlinear terms and filtered joint torques S'*tau - h (Equation 5)
  jointTorques_ = model_->getJointTorques();
  nonlinearityAndTorque_ = selectionMatrixTransposed_*jointTorques_.toImplementation() - nonlinearEffects_;
  /***************************/

  /******************
   * Contact Forces *
   ******************/
  Eigen::VectorXd stackedContactForces;
  if (useWholeBodyContactForces_) {
    // Calculated stacked support jacobian
    StackedSupportJacobian supportJacobianInWorldFrame = StackedSupportJacobian::Zero();
    StackedSupportJacobian supportJacobianDerivativeInWorldFrame = StackedSupportJacobian::Zero();
    auto startRow = 0u;
    for (const auto contactKey : AD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      const auto& contact = model_->getContactContainer().at(contactEnum);

      // Add translational Jacobian.
      Eigen::MatrixXd temp = AD::JacobianTranslation::Zero();
      contact->getJacobianTranslationWorldToContact(
          temp,
          AD::CoordinateFrameEnum::WORLD);
      supportJacobianInWorldFrame.middleRows<AD::getNumTranslationalDof()>(startRow) = temp;
      jacobianTranslationWorldToEndEffectorInWorldFrameVector_[contactEnum] = temp;

      // Add derived translational Jacobian
      contact->getJacobianTranslationTimeDerivativeWorldToContact(
          temp,
          AD::CoordinateFrameEnum::WORLD);
      supportJacobianDerivativeInWorldFrame.middleRows<AD::getNumTranslationalDof()>(startRow) = temp;
      jacobianTranslationTimeDerivativeWorldToEndEffectorInWorldFrameVector_[contactEnum] = temp;

      startRow += AD::getNumTranslationalDof();
    }

    // Compute contact forces assuming contact is closed (script robot dynamics, eq. 3.61)
    const Eigen::MatrixXd jacobianTimesMassMatrixInverse = supportJacobianInWorldFrame*massMatrix_.inverse();
    stackedContactForces = -1.0*(jacobianTimesMassMatrixInverse*supportJacobianInWorldFrame.transpose()).inverse()*(
        jacobianTimesMassMatrixInverse*nonlinearityAndTorque_+
        supportJacobianDerivativeInWorldFrame*generalizedVelocities_
    );
  }
  /******************/





  /******************************
   * Limb dependend coordinates *
   ******************************/
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto branchEnum = AD::mapKeyEnumToKeyEnum<AD::ContactEnum, AD::BranchEnum>(contactEnum);
    const auto limbEnum = AD::mapEnums<AD::LimbEnum>(branchEnum);
    const auto bodyEnum = AD::mapKeyEnumToKeyEnum<AD::ContactEnum, AD::BodyEnum>(contactEnum);
    const auto branchIdInU = AD::getBranchStartIndexInU(branchEnum);
    const auto limbIdInJ = AD::getLimbStartIndexInJ(limbEnum);
    const auto& contact = model_->getContactContainer().at(contactEnum);

    // Get translational Jacobian.
    Eigen::MatrixXd Jmat = AD::JacobianTranslation::Zero();
    contact->getJacobianTranslationWorldToContact(
        Jmat,
        AD::CoordinateFrameEnum::WORLD);
    jacobianTranslationWorldToEndEffectorInWorldFrameVector_[contactEnum] = Jmat;

    // Get derived translational Jacobian.
    Eigen::MatrixXd JDotmat = AD::JacobianTranslation::Zero();
    contact->getJacobianTranslationTimeDerivativeWorldToContact(
        JDotmat,
        AD::CoordinateFrameEnum::WORLD);
    jacobianTranslationTimeDerivativeWorldToEndEffectorInWorldFrameVector_[contactEnum] = JDotmat;

    // Inverted mass matrix of leg.
    legMassMatrixInverted_[contactEnum] = massMatrix_.block(branchIdInU, branchIdInU, numDofLimb_[contactEnum], numDofLimb_[contactEnum]).inverse();

    // Get linear velocity foot.
    linearVelocityEndEffectorInWorldFrameVector_[contactEnum] = model_->getLinearVelocityWorldToBody(bodyEnum, AD::CoordinateFrameEnum::WORLD);

    if (useWholeBodyContactForces_) {
      contactForceInWorldFrameVector_[contactEnum] = stackedContactForces.segment(limbIdInJ, AD::getNumTranslationalDof());
    } else {
      const Eigen::MatrixXd operationalSpaceLegMassMatrix = (
          jacobianTranslationWorldToEndEffectorInWorldFrameVector_[contactEnum].middleCols(branchIdInU, numDofLimb_[contactEnum])
          * legMassMatrixInverted_[contactEnum]
          * jacobianTranslationWorldToEndEffectorInWorldFrameVector_[contactEnum].middleCols(branchIdInU, numDofLimb_[contactEnum]).transpose()
      ).inverse();

      // Compute end-effector force (Equation 6).
      const Eigen::VectorXd estimatedLinearAccelerationLegInWorldFrameInAir = legMassMatrixInverted_[contactEnum] * (
          nonlinearityAndTorque_.segment(branchIdInU, numDofLimb_[contactEnum])
          - massMatrix_.block(branchIdInU, baseIdInU, numDofLimb_[contactEnum], numDofBase)*generalizedAccelerations_.segment<numDofBase>(baseIdInU)
      );

      contactForceInWorldFrameVector_[contactEnum] = -1.0*operationalSpaceLegMassMatrix*(
          jacobianTranslationWorldToEndEffectorInWorldFrameVector_[contactEnum].leftCols(numDofBase)*generalizedAccelerations_.segment<numDofBase>(baseIdInU)
          + jacobianTranslationWorldToEndEffectorInWorldFrameVector_[contactEnum].middleCols(branchIdInU, numDofLimb_[contactEnum])*estimatedLinearAccelerationLegInWorldFrameInAir
          + jacobianTranslationTimeDerivativeWorldToEndEffectorInWorldFrameVector_[contactEnum]*generalizedVelocities_
      );
    }


    // Get end-effector position.
    model_->getPositionWorldToBody(
        positionWorldToEndEffectorInWorldFrameVector_[contactEnum],
        branchEnum,
        AD::BodyNodeEnum::FOOT,
        AD::CoordinateFrameEnum::WORLD);

    // Update contact locations only after a change in the contact state
    const bool isTouchDown = contactState[contactEnum] == ContactState::CLOSED || contactState[contactEnum] == ContactState::SLIPPING;
    if (isTouchDown && contactState[contactEnum]!=previousContactState_[contactEnum]) {
      positionWorldToEndEffectorTouchDownInWorldFrameVector_[contactEnum] = positionWorldToEndEffectorInWorldFrameVector_[contactEnum];
    }
  }
  /******************************/

  if (!updatePlaneEstimation()) { return false; }
  if (!updateExpectedTouchDownPosition(contactState)) { return false; }
  previousContactState_ = contactState;

  return true;
}


bool State::updatePlaneEstimation() {
  /* estimate the plane which best fits the most recent contact points of each foot in world frame
   * using least squares (pseudo inversion of the regressor matrix H)
   *
   * parameters       -> [a b d]^T
   * plane equation   -> z = d-ax-by
   * normal to plane  -> n = [a b 1]^T
   *
   * */
  Eigen::MatrixXd linearRegressor = Eigen::MatrixXd::Zero(numLegs_,3);
  Eigen::VectorXd measuredFootHeights = Eigen::VectorXd::Zero(numLegs_);
  Eigen::Vector3d planeNormalInWorldFrame;
  Eigen::Vector3d positionOnPlaneInWorldFrame;

  // Set up LS problem.
  for (auto footKey : AD::getKeys<anymal_description::AnymalTopology::FootEnum>()) {
    const auto footEnum = footKey.getEnum();
    const auto id = static_cast<unsigned int>(footEnum);
    const auto contactEnum = AD::mapKeyEnumToKeyEnum<anymal_description::AnymalTopology::FootEnum, AD::ContactEnum>(footEnum);

    linearRegressor(id, 0) = -positionWorldToEndEffectorTouchDownInWorldFrameVector_[contactEnum].x();
    linearRegressor(id, 1) = -positionWorldToEndEffectorTouchDownInWorldFrameVector_[contactEnum].y();
    linearRegressor(id, 2) = 1.0;

    measuredFootHeights(id) = positionWorldToEndEffectorTouchDownInWorldFrameVector_[contactEnum].z();
  }

  // Solve LS problem.
  const Eigen::Vector3d parameters = linearRegressor.colPivHouseholderQr().solve(measuredFootHeights);

  // Extract normal.
  planeNormalInWorldFrame << parameters(0), parameters(1), 1.0;
  positionOnPlaneInWorldFrame << 0.0, 0.0, parameters(2);

  // Compute terrain orientation.
  try {
    Eigen::Vector3d axisZ = Eigen::Vector3d::UnitZ();
    orientationWorldToPlane_.setFromVectors(filteredTerrainNormal_.advance(planeNormalInWorldFrame), axisZ).setUnique();
    filteredPositionOnPlane_.advance(positionOnPlaneInWorldFrame);
  } catch (const std::runtime_error& error) {
    MELO_WARN_STREAM(error.what() << "setFromVectors in State::setNormalandPositionInWorldFrame()." << std::endl);
    orientationWorldToPlane_.setIdentity();
    return false;
  }

  return true;
}

bool State::updateExpectedTouchDownPosition(const std_utils::EnumArray<AD::ContactEnum, ContactState>& contactState) {
  // Update expected ground ground height at touch-down.
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    if (contactState[contactEnum] == ContactState::OPEN) {
      expectedGroundHeightInWorldFrame_[contactEnum].advance(getHeight(positionWorldToEndEffectorInWorldFrameVector_[contactEnum]));
    } else {
      expectedGroundHeightInWorldFrame_[contactEnum].advance(positionWorldToEndEffectorInWorldFrameVector_[contactEnum].z());
    }
  }

  return true;
}

const Eigen::MatrixXd& State::getMassMatrix() const noexcept {
  return massMatrix_;
}

const Eigen::Matrix3d& State::getLegMassMatrixInverted(AD::ContactEnum contactEnum) const noexcept {
  return legMassMatrixInverted_[contactEnum];
}

const Eigen::MatrixXd& State::getSelectionMatrixTransposed() const noexcept {
  return selectionMatrixTransposed_;
}

const Eigen::VectorXd& State::getNonlinearityAndTorque() const noexcept {
  return nonlinearityAndTorque_;
}

const Eigen::Vector3d& State::getPositionWorldToEndEffectorInWorldFrame(AD::ContactEnum contactEnum) const noexcept {
  return positionWorldToEndEffectorInWorldFrameVector_[contactEnum];
}

const anymal_model::GeneralizedVelocities& State::getGeneralizedVelocities() const noexcept {
  return generalizedVelocities_;
}

const anymal_model::GeneralizedAccelerations& State::getGeneralizedAccelerations() const noexcept {
  return generalizedAccelerations_;
}

const anymal_model::JointTorques& State::getJointTorques() const noexcept {
  return jointTorques_;
}

const Eigen::Vector3d& State::getLinearVelocityEndEffectorInWorldFrame(AD::ContactEnum contactEnum) const noexcept {
  return linearVelocityEndEffectorInWorldFrameVector_[contactEnum];
}

const Eigen::Vector3d& State::getContactForceInWorldFrame(AD::ContactEnum contactEnum) const noexcept {
  return contactForceInWorldFrameVector_[contactEnum];
}

double State::getExpectedGroundHeightInWorldFrame(AD::ContactEnum contactEnum) const noexcept {
  return expectedGroundHeightInWorldFrame_[contactEnum].getFilteredValue();
}

const kindr::EulerAnglesZyxPD& State::getOrientationWorldToPlane() const noexcept {
  return orientationWorldToPlane_;
}

const Eigen::MatrixXd& State::getJacobianTranslationWorldToEndEffectorInWorldFrame(AD::ContactEnum contactEnum) const noexcept {
  return jacobianTranslationWorldToEndEffectorInWorldFrameVector_[contactEnum];
}

const ContactState& State::getContactState(AD::ContactEnum contactEnum) const {
  return previousContactState_[contactEnum];
}

double State::getHeight(const Eigen::Vector3d& positionWorldToLocationInWorldFrame) const {
  const Eigen::Vector3d& position = filteredPositionOnPlane_.getFilteredValue();
  const Eigen::Vector3d& normal = filteredTerrainNormal_.getFilteredValue();
  const double height = position.z() +
      normal.x()*( position.x()-positionWorldToLocationInWorldFrame.x() ) +
      normal.y()*( position.y()-positionWorldToLocationInWorldFrame.y() );
  return (height/normal.z());
}

} /* namespace contact_estimation */
