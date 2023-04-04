/*
 * AnymalModel.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Dario Bellicoso, Christian Gehring, Peter Fankhauser
 */

// anymal model
#include "anymal_model/AnymalModel.hpp"
#include "anymal_model/rbdl_utils.hpp"

// romo
#include <romo_rbdl/container_utils.hpp>
#include <romo_rbdl/rbdl_utils.hpp>
#include <romo_std/common/container_utils.hpp>

// stl
#include <fstream>

// message logger
#include <message_logger/message_logger.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

// robot utils
#include <robot_utils/math/LinearAlgebra.hpp>
#include <robot_utils/math/OptimizationNelderMead.hpp>
#include <robot_utils/physical_definitions.hpp>

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

namespace anymal_model {

inline bool fileExists(const std::string& pathToFile) {
  return static_cast<bool>(std::ifstream(pathToFile));
}

AnymalModel::AnymalModel(double dt)
    : Base(),
      params_(),
      minHipToFootLength_(0.0),
      maxHipToFootLength_(0.0),
      maxThighToFootLength_(0.0),
      legConfigurations_(true, true, false, false),
      inverseKinematics_(params_),
      isRealRobot_(false),
      timeStep_(dt) {
  limitsPtr_.reset(new LimitsAnymal());
}

double AnymalModel::getHipToFootNorm(const Eigen::VectorXd& legJoints) {
  AnymalState actualState = getState();  // Make copy of actual state.
  AnymalState virtualState = getState();
  JointPositions jointPositions = virtualState.getJointPositions();
  jointPositions.toImplementation().segment<3>(0) = legJoints;
  virtualState.setJointPositions(jointPositions);
  setState(virtualState, true, false, false);
  const double norm = getPositionBodyToBody(BodyEnum::LF_HIP, BodyEnum::LF_FOOT, CoordinateFrameEnum::WORLD).norm();
  setState(actualState, true, false, false);  // Reset model to actual state.
  return norm;
}

double AnymalModel::getThighToFootNorm(const Eigen::VectorXd& legJoints) {
  AnymalState actualState = getState();  // Make copy of actual state.
  AnymalState virtualState = getState();
  JointPositions jointPositions = virtualState.getJointPositions();
  jointPositions.toImplementation().segment<3>(0) = legJoints;
  virtualState.setJointPositions(jointPositions);
  setState(virtualState, true, false, false);
  const double norm = getPositionBodyToBody(BodyEnum::LF_THIGH, BodyEnum::LF_FOOT, CoordinateFrameEnum::WORLD).norm();
  setState(actualState, true, false, false);  // Reset model to actual state.
  return norm;
}

double AnymalModel::getLegGravityNorm(const Eigen::VectorXd& legJoints) {
  AnymalState actualState = getState();  // Make copy of actual state.
  AnymalState virtualState = getState();
  JointPositions jointPositions = virtualState.getJointPositions();
  jointPositions.toImplementation().segment<3>(0) = legJoints;
  virtualState.setJointPositions(jointPositions);
  setState(virtualState, true, false, false);
  const double norm = getGravityTerms().segment<3>(6).norm();
  setState(actualState, true, false, false);  // Reset model to actual state.
  return norm;
}

const Eigen::MatrixXd& AnymalModel::getActuatorSelectionMatrix() const {
  return actuatorSelectionMatrix_;
}

void AnymalModel::setIsRealRobot(bool isRealRobot) {
  isRealRobot_ = isRealRobot;
}

bool AnymalModel::getIsRealRobot() const {
  return isRealRobot_;
}

bool AnymalModel::initializeFromUrdfImpl(const std::string& urdfString, bool verbose) {
  //! Initialize model
  if (!Base::initializeFromUrdfImpl(urdfString, verbose)) {
    MELO_WARN_STREAM("[AnymalModel::initializeFromUrdf] Initializing base model failed.");
    return false;
  }

  // Initialize the limits
  if (!initLimitsFromUrdfDescription(this->getUrdfDescription())) {
    MELO_WARN_STREAM("[AnymalModel::initializeFromUrdf] Initializing limits failed.");
    return false;
  }

  // Set the initial state to zero
  setRbdlQFromState(stateGeneralizedPositionsQuaternionRBDL_, state_);
  setRbdlQDotFromState(stateGeneralizedVelocitiesAngularRBDL_, state_);
  updateKinematics(true, true, false);

  // Initialize kinematic parameters
  Eigen::Vector3d position;

  // Loop over all limbs to set parameters.
  for (const auto& limbKey : AD::getLimbKeys()) {
    const auto limb = limbKey.getEnum();
    /* main body */
    position =
        getPositionBodyToBody(BodyEnum::BASE, anymal_description::AnymalDefinitions::mapLimbToHip::at(limb), CoordinateFrameEnum::BASE);
    params_.setPositionBaseToHipInBaseFrame(Position(position), limb);

    /* hip */
    position = getPositionBodyToBody(anymal_description::AnymalDefinitions::mapLimbToHip::at(limb),
                                     anymal_description::AnymalDefinitions::mapLimbToThigh::at(limb), CoordinateFrameEnum::BASE);
    params_.setPositionHipToThighInHipFrame(Position(position), limb);

    /* thigh */
    position = getPositionBodyToBody(anymal_description::AnymalDefinitions::mapLimbToThigh::at(limb),
                                     anymal_description::AnymalDefinitions::mapLimbToShank::at(limb), CoordinateFrameEnum::BASE);
    params_.setPositionThighToShankInThighFrame(Position(position), limb);

    /* shank */
    position = getPositionBodyToBody(anymal_description::AnymalDefinitions::mapLimbToShank::at(limb),
                                     anymal_description::AnymalDefinitions::mapLimbToFoot::at(limb), CoordinateFrameEnum::BASE);
    params_.setPositionShankToFootInShankFrame(Position(position), limb);
  }

  // Initialize parameters to precompute derived ones
  params_.initialize();

  // Find the leg workspace
  robot_utils::optimization::OptimizationNelderMead optNm;
  Eigen::VectorXd initialGuess(3);
  Eigen::VectorXd minJoints(3);
  Eigen::VectorXd maxJoints(3);
  Eigen::VectorXd maxThighJoints(3);
  Eigen::VectorXd minGravityJoints(3);
  initialGuess.setZero();
  optNm.optimize(minHipToFootLength_, minJoints, initialGuess, std::bind(&AnymalModel::getHipToFootNorm, this, std::placeholders::_1), true,
                 1000);

  optNm.optimize(maxHipToFootLength_, maxJoints, initialGuess, std::bind(&AnymalModel::getHipToFootNorm, this, std::placeholders::_1),
                 false, 1000);

  optNm.optimize(maxThighToFootLength_, maxThighJoints, initialGuess,
                 std::bind(&AnymalModel::getThighToFootNorm, this, std::placeholders::_1), false, 1000);

  double minGravity;
  optNm.optimize(minGravity, minGravityJoints, initialGuess, std::bind(&AnymalModel::getLegGravityNorm, this, std::placeholders::_1), true,
                 1000);
  //  MELO_INFO_STREAM("[AnymalModel::initializeFromUrdf] Found min gravity: " << minGravity << " at joint pos: " << minGravityJoints);

  inverseKinematics_.initialize(minHipToFootLength_, maxHipToFootLength_, maxThighToFootLength_);

  actuatorSelectionMatrix_ = Eigen::Matrix<double, AD::getActuatorsDimension(), AD::getNumDof()>::Zero();
  actuatorSelectionMatrix_.bottomRightCorner(AD::getActuatorsDimension(), AD::getActuatorsDimension()).setIdentity();

  return true;
}

bool AnymalModel::initLimitsFromUrdfDescription(const std::string& urdfDescription) {
  // Create the ordered sequence of joint names from the topology // Do we need this?
  std::vector<std::string> joints(AD::getJointKeys().size());
  for (const auto& jointKey : AD::getJointKeys()) {
    joints[jointKey.getId()] = jointKey.getName();
  }

  // Initialize Limits.
  limitsPtr_->init();

  // Get handle to urdf and robot.
  TiXmlDocument urdfDocument;
  urdfDocument.Parse(urdfDescription.c_str());
  const TiXmlHandle urdfHandle(&urdfDocument);
  TiXmlElement* robotElement = urdfDocument.FirstChildElement("robot");

  // Get all joint elements.
  std::vector<TiXmlElement*> jointElements;
  if (!tinyxml_tools::getChildElements(jointElements, robotElement, "joint")) {
    MELO_WARN_STREAM("[AnymalModel::initLimitsFromUrdfString] Could not get joint elements.")
  }
  std::unordered_map<std::string, TiXmlElement*> jointElementsByName;
  for (auto& jointElement : jointElements) {
    const char* name = jointElement->Attribute("name");
    if (name != nullptr) {
      jointElementsByName.emplace(std::make_pair(static_cast<std::string>(name), jointElement));
    }
  }

  double lower, upper, velocity, effort, commandEffort, gearVelocity, current;

  // Read limit values from joint elements.
  for (const auto& jointKey : AD::getJointKeys()) {
    const auto& jointName = jointKey.getName();
    const auto jointEnum = jointKey.getEnum();
    const AD::ActuatorEnum actuatorEnum = AD::mapKeyEnumToKeyEnum<AD::JointEnum, AD::ActuatorEnum>(jointEnum);

    const auto jointIterator = jointElementsByName.find(jointName);
    if (jointIterator == jointElementsByName.end()) {
      MELO_WARN_STREAM("[AnymalModel::initLimitsFromUrdfString] Could not find " << jointName << " joint in urdf description.");
      continue;
    }

    // A convenience lambda for reading double parameters (C++14 would be required to have "auto" instead of "double").
    auto loadParameterFromUrdf = [&](double& parameter, const std::string& parameterName) -> bool {
      if (!tinyxml_tools::loadParameter(parameter, jointIterator->second->FirstChildElement("limit"), parameterName)) {
        MELO_WARN_STREAM("[AnymalModel::initLimitsFromUrdfString] Could not load parameter " << parameterName << " of " << jointName
                                                                                             << " joint from urdf description.");
        return false;
      }
      return true;
    };

    if (loadParameterFromUrdf(lower, "lower")) {
      getLimitsAnymal()->setJointMinPosition(jointEnum, lower);
      getLimitsAnymal()->setActuatorMinPosition(actuatorEnum, lower);
    }
    if (loadParameterFromUrdf(upper, "upper")) {
      getLimitsAnymal()->setJointMaxPosition(jointEnum, upper);
      getLimitsAnymal()->setActuatorMaxPosition(actuatorEnum, upper);
    }
    if (loadParameterFromUrdf(velocity, "velocity")) {
      getLimitsAnymal()->setJointMinVelocity(jointEnum, -velocity);
      getLimitsAnymal()->setActuatorMinVelocity(actuatorEnum, -velocity);
      getLimitsAnymal()->setJointMaxVelocity(jointEnum, velocity);
      getLimitsAnymal()->setActuatorMaxVelocity(actuatorEnum, velocity);
    }
    if (loadParameterFromUrdf(effort, "effort")) {
      getLimitsAnymal()->setJointMinEffort(jointEnum, -effort);
      getLimitsAnymal()->setActuatorMinEffort(actuatorEnum, -effort);
      getLimitsAnymal()->setJointMaxEffort(jointEnum, effort);
      getLimitsAnymal()->setActuatorMaxEffort(actuatorEnum, effort);
    }
    if (loadParameterFromUrdf(commandEffort, "command_effort")) {
      getLimitsAnymal()->setJointMinCommandEffort(jointEnum, -commandEffort);
      getLimitsAnymal()->setActuatorMinCommandEffort(actuatorEnum, -commandEffort);
      getLimitsAnymal()->setJointMaxCommandEffort(jointEnum, commandEffort);
      getLimitsAnymal()->setActuatorMaxCommandEffort(actuatorEnum, commandEffort);
    }
    if (loadParameterFromUrdf(gearVelocity, "gear_velocity")) {
      getLimitsAnymal()->setActuatorMinGearVelocity(actuatorEnum, -gearVelocity);
      getLimitsAnymal()->setActuatorMaxGearVelocity(actuatorEnum, gearVelocity);
    }
    if (loadParameterFromUrdf(current, "current")) {
      getLimitsAnymal()->setActuatorMinCurrent(actuatorEnum, -current);
      getLimitsAnymal()->setActuatorMaxCurrent(actuatorEnum, current);
    }
  }

  return true;
}

const AnymalParameters& AnymalModel::getParameters() const {
  return params_;
}

bool AnymalModel::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(Eigen::Vector3d& legJoints,
                                                                         const Eigen::Vector3d& positionBaseToFootInBaseFrame, int leg) {
  AT::LegConfigEnum legConfig;

  const auto limb = AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg);

  if (anymal_description::AnymalDefinitions::mapLimbToLongitudinal::at(limb) == AT::LongitudinalEnum::FORE) {
    legConfig = getLegConfigurations()[leg] ? AT::LegConfigEnum::XConfiguration : AT::LegConfigEnum::OConfiguration;
  } else {
    legConfig = getLegConfigurations()[leg] ? AT::LegConfigEnum::OConfiguration : AT::LegConfigEnum::XConfiguration;
  }

  return inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(legJoints, positionBaseToFootInBaseFrame, limb,
                                                                                   legConfig);
}

bool AnymalModel::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(Eigen::Vector3d& legJoints,
                                                                         const Eigen::Vector3d& positionBaseToFootInBaseFrame,
                                                                         LimbEnum limb) {
  AT::LegConfigEnum legConfig;

  const auto legId = AD::mapKeyEnumToKeyId(limb);

  if (anymal_description::AnymalDefinitions::mapLimbToLongitudinal::at(limb) == AT::LongitudinalEnum::FORE) {
    legConfig = getLegConfigurations()[legId] ? AT::LegConfigEnum::XConfiguration : AT::LegConfigEnum::OConfiguration;
  } else {
    legConfig = getLegConfigurations()[legId] ? AT::LegConfigEnum::OConfiguration : AT::LegConfigEnum::XConfiguration;
  }

  return inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(legJoints, positionBaseToFootInBaseFrame, limb,
                                                                                   legConfig);
}

const JointAccelerations& AnymalModel::getJointAccelerations() const {
  return state_.getJointAccelerations();
}

void AnymalModel::setJointAccelerations(const JointAccelerations& jointAccelerations) {
  state_.setJointAccelerations(jointAccelerations);
}

const JointTorques& AnymalModel::getJointTorques() const {
  return state_.getJointTorques();
}

void AnymalModel::setJointTorques(const JointTorques& jointTorques) {
  state_.setJointTorques(jointTorques);
}

double AnymalModel::getMinHipToFootLength() const {
  return minHipToFootLength_;
}

double AnymalModel::getMaxHipToFootLength() const {
  return maxHipToFootLength_;
}

double AnymalModel::getTimeStep() const {
  return timeStep_;
}

const LegConfigurations& AnymalModel::getLegConfigurations() const {
  return legConfigurations_;
}

void AnymalModel::setLegConfigurations(const LegConfigurations& configurations) {
  legConfigurations_ = configurations;
}

void AnymalModel::getGeneralizedPositionsFromSymmetricJointPositions(Eigen::VectorXd& generalizedPositions, double leftForeHaa,
                                                                     double leftForeHfe, double leftForeKfe) {
  // fixme: use enums
  generalizedPositions(6) = leftForeHaa;
  generalizedPositions(7) = (legConfigurations_[0] ? leftForeHfe : -leftForeHfe);
  generalizedPositions(8) = (legConfigurations_[0] ? leftForeKfe : -leftForeKfe);

  generalizedPositions(9) = -leftForeHaa;
  generalizedPositions(10) = (legConfigurations_[1] ? leftForeHfe : -leftForeHfe);
  generalizedPositions(11) = (legConfigurations_[1] ? leftForeKfe : -leftForeKfe);

  generalizedPositions(12) = -leftForeHaa;
  generalizedPositions(13) = (legConfigurations_[2] ? leftForeHfe : -leftForeHfe);
  generalizedPositions(14) = (legConfigurations_[2] ? leftForeKfe : -leftForeKfe);

  generalizedPositions(15) = -leftForeHaa;
  generalizedPositions(16) = (legConfigurations_[3] ? leftForeHfe : -leftForeHfe);
  generalizedPositions(17) = (legConfigurations_[3] ? leftForeKfe : -leftForeKfe);
}

void AnymalModel::addVariablesToLog(bool /*update*/, const std::string& ns) {
  signal_logger::add(state_.getPositionWorldToBaseInWorldFrame(), "positionWorldToBaseInWorldFrame", ns);
  signal_logger::add(state_.getOrientationBaseToWorld(), "orientationBaseToWorld", ns);
  signal_logger::add(state_.getJointPositions()(0), "jointPositions/LF_HAA", ns);
  signal_logger::add(state_.getJointPositions()(1), "jointPositions/LF_HFE", ns);
  signal_logger::add(state_.getJointPositions()(2), "jointPositions/LF_KFE", ns);
  signal_logger::add(state_.getJointPositions()(3), "jointPositions/RF_HAA", ns);
  signal_logger::add(state_.getJointPositions()(4), "jointPositions/RF_HFE", ns);
  signal_logger::add(state_.getJointPositions()(5), "jointPositions/RF_KFE", ns);
  signal_logger::add(state_.getJointPositions()(6), "jointPositions/LH_HAA", ns);
  signal_logger::add(state_.getJointPositions()(7), "jointPositions/LH_HFE", ns);
  signal_logger::add(state_.getJointPositions()(8), "jointPositions/LH_KFE", ns);
  signal_logger::add(state_.getJointPositions()(9), "jointPositions/RH_HAA", ns);
  signal_logger::add(state_.getJointPositions()(10), "jointPositions/RH_HFE", ns);
  signal_logger::add(state_.getJointPositions()(11), "jointPositions/RH_KFE", ns);

  signal_logger::add(state_.getLinearVelocityBaseInWorldFrame(), "linearVelocityBaseInWorldFrame", ns);
  signal_logger::add(state_.getAngularVelocityBaseInBaseFrame(), "angularVelocityBaseInBaseFrame", ns);
  signal_logger::add(state_.getJointVelocities()(0), "jointVelocities/LF_HAA", ns);
  signal_logger::add(state_.getJointVelocities()(1), "jointVelocities/LF_HFE", ns);
  signal_logger::add(state_.getJointVelocities()(2), "jointVelocities/LF_KFE", ns);
  signal_logger::add(state_.getJointVelocities()(3), "jointVelocities/RF_HAA", ns);
  signal_logger::add(state_.getJointVelocities()(4), "jointVelocities/RF_HFE", ns);
  signal_logger::add(state_.getJointVelocities()(5), "jointVelocities/RF_KFE", ns);
  signal_logger::add(state_.getJointVelocities()(6), "jointVelocities/LH_HAA", ns);
  signal_logger::add(state_.getJointVelocities()(7), "jointVelocities/LH_HFE", ns);
  signal_logger::add(state_.getJointVelocities()(8), "jointVelocities/LH_KFE", ns);
  signal_logger::add(state_.getJointVelocities()(9), "jointVelocities/RH_HAA", ns);
  signal_logger::add(state_.getJointVelocities()(10), "jointVelocities/RH_HFE", ns);
  signal_logger::add(state_.getJointVelocities()(11), "jointVelocities/RH_KFE", ns);

  signal_logger::add(state_.getJointTorques()(0), "jointTorques/LF_HAA", ns);
  signal_logger::add(state_.getJointTorques()(1), "jointTorques/LF_HFE", ns);
  signal_logger::add(state_.getJointTorques()(2), "jointTorques/LF_KFE", ns);
  signal_logger::add(state_.getJointTorques()(3), "jointTorques/RF_HAA", ns);
  signal_logger::add(state_.getJointTorques()(4), "jointTorques/RF_HFE", ns);
  signal_logger::add(state_.getJointTorques()(5), "jointTorques/RF_KFE", ns);
  signal_logger::add(state_.getJointTorques()(6), "jointTorques/LH_HAA", ns);
  signal_logger::add(state_.getJointTorques()(7), "jointTorques/LH_HFE", ns);
  signal_logger::add(state_.getJointTorques()(8), "jointTorques/LH_KFE", ns);
  signal_logger::add(state_.getJointTorques()(9), "jointTorques/RH_HAA", ns);
  signal_logger::add(state_.getJointTorques()(10), "jointTorques/RH_HFE", ns);
  signal_logger::add(state_.getJointTorques()(11), "jointTorques/RH_KFE", ns);
}

void AnymalModel::setState(const AnymalState& state, bool updatePosition, bool updateVelocity, bool updateAcceleration) {
  state_ = state;
  setRbdlQFromState(stateGeneralizedPositionsQuaternionRBDL_, state_);
  setRbdlQDotFromState(stateGeneralizedVelocitiesAngularRBDL_, state_);
  updateKinematics(updatePosition, updateVelocity, updateAcceleration);
}

}  // namespace anymal_model
