/*
 * PointFootAnymal.cpp
 *
 *  Created on: Jan 10, 2017
 *      Author: Dario Bellicoso, Gabriel Hottiger
 */

// loco anymal
#include "loco_anymal/common/PointFootAnymal.hpp"

// loco
#include "loco/common/end_effectors/EndEffectorProperties.hpp"

// robot utils
#include "robot_utils/math/LinearAlgebra.hpp"


namespace loco_anymal {

PointFootAnymal::PointFootAnymal(
    ContactEnum contactEnum, const std::string& name,
    anymal_model::AnymalModel& anymalModel,
    anymal_model::AnymalModel& anymalModelDesired)
    : FootRomo(AD::mapEnums<AD::BodyEnum>(contactEnum),
               name, anymalModel, { {loco::EndEffectorEnum::Origin, contactEnum} }),
      anymalModel_(anymalModel),
      anymalModelDesired_(anymalModelDesired)
{ }

PointFootAnymal::PointFootAnymal(
    ContactEnum contactEnum, const std::string& name,
    anymal_model::AnymalModel& anymalModel,
    anymal_model::AnymalModel& anymalModelDesired,
    loco::EndEffectorPropertiesPtr&& endEffectorProperties)
    : FootRomo(AD::mapEnums<AD::BodyEnum>(contactEnum),
               name, anymalModel, std::move(endEffectorProperties), { {loco::EndEffectorEnum::Origin, contactEnum} }),
      anymalModel_(anymalModel),
      anymalModelDesired_(anymalModelDesired)
{ }

JointPositions PointFootAnymal::getJointPositionsFromPositionBaseToEndEffectorInBaseFrameIteratively(
    const loco::Position& positionBaseToEndEffectorInBaseFrame)
{
  using RD = AD;
  Eigen::VectorXd legJoints;
  anymalModel_.getLimbJointPositionsFromLimbEnumIteratively(
      legJoints, positionBaseToEndEffectorInBaseFrame.toImplementation(),
      RD::mapEnums<RD::LimbEnum>(branchEnum_));
  return JointPositions(legJoints);
}

JointPositions PointFootAnymal::getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(
    const loco::Position& positionBaseToFootInBaseFrame)
{
  using RD = AD;
  Eigen::Vector3d legJoints;
  anymalModel_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      legJoints, positionBaseToFootInBaseFrame.toImplementation(), RD::mapKeyEnumToKeyId(limbEnum_));
  return JointPositions(legJoints);
}

JointVelocities PointFootAnymal::getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(
    const loco::LinearVelocity& velocity)
{
  return JointVelocities(robot_utils::pseudoInverseAdaptiveDls(
      getStateMeasured().getTranslationJacobianBaseToEndEffectorInBaseFrame())
    * velocity.toImplementation()
  );
}

loco::Position PointFootAnymal::getPositionWorldToEndEffectorInWorldFrame(const JointPositions& legJoints)
{
  using RD = AD;
  anymal_model::AnymalState anymalState;
  anymal_model::JointPositions jointPositions;
  jointPositions.toImplementation().segment<RD::getNumDofLimb()>(RD::getLimbStartIndexInJ(limbEnum_)) = legJoints;
  anymalState.setJointPositions(jointPositions);
  anymalModelDesired_.setState(anymalState, true, false, false);

  return loco::Position(
      anymalModelDesired_.getPositionWorldToBody(
          branchEnum_, RD::BodyNodeEnum::FOOT, RD::CoordinateFrameEnum::WORLD)
  );
}

loco::Position PointFootAnymal::getPositionWorldToEndEffectorInBaseFrame(const JointPositions& legJoints)
{
  using RD = AD;
  anymal_model::AnymalState anymalState;
  anymal_model::JointPositions jointPositions;
  jointPositions.toImplementation().segment<RD::getNumDofLimb()>(RD::getLimbStartIndexInJ(limbEnum_)) = legJoints;
  anymalState.setJointPositions(jointPositions);
  anymalModelDesired_.setState(anymalState, true, false, false);

  return loco::Position(
      anymalModelDesired_.getPositionWorldToBody(
          branchEnum_, RD::BodyNodeEnum::FOOT, RD::CoordinateFrameEnum::BASE)
  );
}

loco::Position PointFootAnymal::getPositionBaseToEndEffectorInBaseFrame(const JointPositions& legJoints)
{
  using RD = AD;
  anymal_model::AnymalState anymalState;
  anymal_model::JointPositions jointPositions;
  jointPositions.toImplementation().segment<RD::getNumDofLimb()>(RD::getLimbStartIndexInJ(limbEnum_)) = legJoints;
  anymalState.setJointPositions(jointPositions);

  anymalModelDesired_.setState(anymalState, true, false, false);

  return loco::Position(
      anymalModelDesired_.getPositionBodyToBody(
          RD::BodyEnum::BASE, branchEnum_, RD::BodyNodeEnum::FOOT, RD::CoordinateFrameEnum::BASE)
  );
}

Eigen::MatrixXd PointFootAnymal::getTranslationJacobianBaseToEndEffectorInBaseFrameFromJointAngles(const JointPositions& legJoints) {
  anymal_model::AnymalState anymalState;
  anymal_model::JointPositions jointPositions;
  anymal_model::AnymalState anymalStateOriginal = anymalModelDesired_.getState();

  // Compute the Jacobian from the given joint angles.
  jointPositions.toImplementation().segment<AD::getNumDofLimb()>(AD::getLimbStartIndexInJ(limbEnum_)) = legJoints;
  anymalState.setJointPositions(jointPositions);
  anymalModelDesired_.setState(anymalState, true, false, false);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(AD::getNumTranslationalDof(), AD::getGeneralizedVelocitiesDimension());
  anymalModelDesired_.getJacobianTranslationFloatingBaseToBody(jacobian, branchEnum_, AD::BodyNodeEnum::FOOT,
                                                               AD::CoordinateFrameEnum::BASE);

  // Reset the model.
  anymalModelDesired_.setState(anymalStateOriginal, true, false, false);

  return jacobian;
}
} /* namespace loco_anymal */
