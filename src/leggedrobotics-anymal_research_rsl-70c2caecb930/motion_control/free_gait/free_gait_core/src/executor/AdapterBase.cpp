/*
 * AdapterBase.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/executor/AdapterBase.hpp>

namespace free_gait {

AdapterBase::AdapterBase()
{
}

AdapterBase::~AdapterBase()
{
}

bool AdapterBase::frameIdExists(const std::string& frameId) const
{
  if (frameId == getBaseFrameId()) return true;
  if (frameId == getWorldFrameId()) return true;
  return false;
}

Position AdapterBase::transformPosition(const std::string& inputFrameId,
                                        const std::string& outputFrameId,
                                        const Position& position) const
{
  Position transformedPosition;
  Pose outputFramePose, inputFramePose;
  bool frameError = false;

  if (inputFrameId == outputFrameId) {
    return position;
  }

  if (inputFrameId == getBaseFrameId()) {

    if (outputFrameId == getWorldFrameId()) {
      transformedPosition = getPositionWorldToBaseInWorldFrame() + getOrientationBaseToWorld().rotate(position);
    } else if (getFrameTransform(outputFrameId, outputFramePose)) {
      const Position positionInWorld = getPositionWorldToBaseInWorldFrame() + getOrientationBaseToWorld().rotate(position);
      transformedPosition = outputFramePose.inverseTransform(positionInWorld);
    } else {
      frameError = true;
    }

  } else if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == getBaseFrameId()) {
      transformedPosition = getOrientationBaseToWorld().inverseRotate(position - getPositionWorldToBaseInWorldFrame());
    } else if (getFrameTransform(outputFrameId, outputFramePose)) {
      transformedPosition = outputFramePose.inverseTransform(position);
    } else {
      frameError = true;
    }

  } else if (getFrameTransform(inputFrameId, inputFramePose)) {

    if (outputFrameId == getBaseFrameId()) {
      const Position positionInWorld = inputFramePose.transform(position);
      transformedPosition = getOrientationBaseToWorld().inverseRotate(positionInWorld - getPositionWorldToBaseInWorldFrame());
    } else if (outputFrameId == getWorldFrameId()) {
      transformedPosition = inputFramePose.transform(position);
    } else if (getFrameTransform(outputFrameId, outputFramePose)) {
      const Position positionInWorld = inputFramePose.transform(position);
      transformedPosition = outputFramePose.inverseTransform(positionInWorld);
    } else {
      frameError = true;
    }
  } else {
    frameError = true;
  }

  if (frameError) {
    const std::string message = "Invalid frame for transforming position (input frame: " + inputFrameId + ", output frame: " + outputFrameId + ").";
    throw std::invalid_argument(message);
  }
  return transformedPosition;
}

RotationQuaternion AdapterBase::transformOrientation(const std::string& inputFrameId,
                                                     const std::string& outputFrameId,
                                                     const RotationQuaternion& orientation) const
{
  RotationQuaternion transformedOrientation;
  Pose outputFramePose, inputFramePose;
  bool frameError = false;

  if (inputFrameId == outputFrameId) {
    return orientation;
  }

  if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == getBaseFrameId()) {
      transformedOrientation = getOrientationBaseToWorld().inverted() * orientation ;
    } else if (getFrameTransform(outputFrameId, outputFramePose)) {
      transformedOrientation = outputFramePose.getRotation().inverted() * orientation;
    } else {
      frameError = true;
    }

  } else if (inputFrameId == getBaseFrameId()) {

    if (outputFrameId == getWorldFrameId()) {
      transformedOrientation = getOrientationBaseToWorld() * orientation;
    } else if (getFrameTransform(outputFrameId, outputFramePose)) {
      const RotationQuaternion orientationToWorld = getOrientationBaseToWorld() * orientation;
      transformedOrientation = outputFramePose.getRotation().inverted() * orientationToWorld;
    } else {
      frameError = true;
    }

  } else if (getFrameTransform(inputFrameId, inputFramePose)) {

    if (outputFrameId == getWorldFrameId()) {
      transformedOrientation = inputFramePose.getRotation() * orientation;
    } else if (outputFrameId == getBaseFrameId()) {
      const RotationQuaternion orientationToWorld = getOrientationBaseToWorld() * orientation;
      transformedOrientation =  inputFramePose.getRotation().inverted() * orientationToWorld;
    } else if (getFrameTransform(outputFrameId, outputFramePose)) {
      const RotationQuaternion orientationToWorld = inputFramePose.getRotation() * orientation;
      transformedOrientation = outputFramePose.getRotation().inverted() * orientationToWorld;
    } else {
      frameError = true;
    }

  } else {
    frameError = true;
  }

  if (frameError) {
    const std::string message = "Invalid frame for transforming orientation (input frame: " + inputFrameId + ", output frame: " + outputFrameId + ").";
    throw std::invalid_argument(message);
  }
  return transformedOrientation;
}

Pose AdapterBase::transformPose(const std::string& inputFrameId, const std::string& outputFrameId,
                                const Pose& pose) const
{
  Pose transformedPose;
  transformedPose.getPosition() = transformPosition(inputFrameId, outputFrameId, pose.getPosition());
  transformedPose.getRotation() = transformOrientation(inputFrameId, outputFrameId, pose.getRotation());
  return transformedPose;
}

LinearVelocity AdapterBase::transformLinearVelocity(const std::string& inputFrameId,
                                                    const std::string& outputFrameId,
                                                    const LinearVelocity& linearVelocity) const
{
  LinearVelocity transformedLinearVelocity(
      transformVector(inputFrameId, outputFrameId, Vector(linearVelocity)));
  return transformedLinearVelocity;
}

LocalAngularVelocity AdapterBase::transformAngularVelocity(
    const std::string& inputFrameId, const std::string& outputFrameId,
    const LocalAngularVelocity& angularVelocity) const
{
  Vector transformedVector = transformVector(inputFrameId, outputFrameId, Vector(angularVelocity.vector()));
  LocalAngularVelocity transformedAngularVelocity(transformedVector.toImplementation());
  return transformedAngularVelocity;
}

Twist AdapterBase::transformTwist(const std::string& inputFrameId, const std::string& outputFrameId,
                                  const Twist& twist) const
{
  Twist transformedTwist;
  transformedTwist.getTranslationalVelocity() = transformLinearVelocity(
      inputFrameId, outputFrameId, twist.getTranslationalVelocity());
  transformedTwist.getRotationalVelocity() = transformAngularVelocity(
      inputFrameId, outputFrameId, twist.getRotationalVelocity());
  return transformedTwist;
}

LinearAcceleration AdapterBase::transformLinearAcceleration(const std::string& inputFrameId,
                                                            const std::string& outputFrameId,
                                                            const LinearAcceleration& linearAcceleration) const
{
  LinearAcceleration transformedLinearAcceleration(
      transformVector(inputFrameId, outputFrameId, Vector(linearAcceleration)));
  return transformedLinearAcceleration;
}

Force AdapterBase::transformForce(const std::string& inputFrameId,
                                  const std::string& outputFrameId,
                                  const Force& force) const
{
  Force transformedForce(
      transformVector(inputFrameId, outputFrameId, Vector(force)));
  return transformedForce;
}

Vector AdapterBase::transformVector(const std::string& inputFrameId,
                                    const std::string& outputFrameId, const Vector& vector) const
{
  Vector transformedVector;
  Pose outputFramePose, inputFramePose;
  bool frameError = false;

  if (inputFrameId == outputFrameId) {
    return vector;
  }

  if (inputFrameId == getBaseFrameId()) {

    if (outputFrameId == getWorldFrameId()) {
      transformedVector = getOrientationBaseToWorld().rotate(vector);
    } else if (getFrameTransform(outputFrameId, outputFramePose)) {
      const Vector vectorInWorld = getOrientationBaseToWorld().rotate(vector);
      transformedVector = outputFramePose.getRotation().inverseRotate(vectorInWorld);
    } else {
      frameError = true;
    }

  } else if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == getBaseFrameId()) {
      transformedVector = getOrientationBaseToWorld().inverseRotate(vector);
    } else if (getFrameTransform(outputFrameId, outputFramePose)) {
      transformedVector = outputFramePose.getRotation().inverseRotate(vector);
    } else {
      frameError = true;
    }

  } else if (getFrameTransform(inputFrameId, inputFramePose)) {

    if (outputFrameId == getBaseFrameId()) {
      const Vector vectorInWorld = inputFramePose.getRotation().rotate(vector);
      transformedVector = getOrientationBaseToWorld().inverseRotate(vectorInWorld);
    } else if (outputFrameId == getWorldFrameId()) {
      transformedVector = inputFramePose.getRotation().rotate(vector);
    } else if (getFrameTransform(outputFrameId, outputFramePose)) {
      const Vector vectorInWorld = inputFramePose.getRotation().rotate(vector);
      transformedVector = outputFramePose.getRotation().inverseRotate(vectorInWorld);
    } else {
      frameError = true;
    }

  } else {
    frameError = true;
  }

  if (frameError) {
    const std::string message = "Invalid frame for transforming vector (input frame: " + inputFrameId + ", output frame: " + outputFrameId + ").";
    throw std::invalid_argument(message);
  }
  return transformedVector;
}

loco::WholeBody* AdapterBase::getWholeBodyPtr() const {
  return nullptr;
}

} /* namespace free_gait */
