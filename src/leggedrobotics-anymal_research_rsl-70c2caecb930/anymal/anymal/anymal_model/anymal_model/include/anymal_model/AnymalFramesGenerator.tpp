/*
 * AnymalFramesGenerator.tpp
 *
 *  Created on: Feb 27, 2018
 *      Author: Dario Bellicoso
 */

// message logger
#include <message_logger/message_logger.hpp>

namespace anymal_model {

template <typename ConcreteAnymalDescription_, typename AnymalState_>
void AnymalFramesGenerator<ConcreteAnymalDescription_, AnymalState_>::resetDerived() {
  for (auto& positionWorldToFootInWorldFrame : positionsWorldToFootInWorldFrame_) {
    positionWorldToFootInWorldFrame.setZero();
  }
  allPositionsWorldToFootInWorldFrameInitialized_ = false;
}

template <typename ConcreteAnymalDescription_, typename AnymalState_>
void AnymalFramesGenerator<ConcreteAnymalDescription_, AnymalState_>::update(const RobotModel& model) {
  kindr::Position3D positionWorldToFootprintInWorldFrame;
  kindr::Position3D positionWorldToFeetcenterInWorldFrame;
  kindr::RotationQuaternionD orientationFootprintToWorld;
  kindr::RotationQuaternionD orientationFeetcenterToWorld;

  for (const auto footKey : AD::template getKeys<FootEnum>()) {
    const auto footEnum = footKey.getEnum();
    const auto branch = AD::template mapEnums<BranchEnum>(footEnum);
    const auto contact = AD::template mapEnums<ContactEnum>(footEnum);

    kindr::Position3D positionWorldToFootInWorldFrame;
    model.getPositionWorldToBody(positionWorldToFootInWorldFrame.toImplementation(), branch, AD::BodyNodeEnum::FOOT,
                                 AD::CoordinateFrameEnum::WORLD);

    positionWorldToFeetcenterInWorldFrame += positionWorldToFootInWorldFrame;

    if (model.getContactContainer()[contact]->getState() == AD::ContactStateEnum::CLOSED) {
      positionsWorldToFootInWorldFrame_[footEnum] = positionWorldToFootInWorldFrame;
    }

    positionWorldToFootprintInWorldFrame += positionsWorldToFootInWorldFrame_[footEnum];
  }
  positionWorldToFootprintInWorldFrame /= static_cast<double>(positionsWorldToFootInWorldFrame_.size());
  positionWorldToFeetcenterInWorldFrame /= static_cast<double>(positionsWorldToFootInWorldFrame_.size());

  // If not initialized yet, check if all positions have been initialized now.
  if (!allPositionsWorldToFootInWorldFrameInitialized_) {
    allPositionsWorldToFootInWorldFrameInitialized_ = true;
    for (const auto positionWorldToFootInWorldFrame : positionsWorldToFootInWorldFrame_) {
      // If one of the positions still contains zero, it has not been initialized yet.
      if (positionWorldToFootInWorldFrame.toImplementation().isZero()) {
        allPositionsWorldToFootInWorldFrameInitialized_ = false;
        break;
      }
    }
  }

  // Only start computing the footprint once all positions have been initialized.
  if (allPositionsWorldToFootInWorldFrameInitialized_) {
    // Find footprint orientation.
    kindr::Position3D headingDirectionFootprintInWorldFrame;
    getHeadingDirectionFootprint(headingDirectionFootprintInWorldFrame, model);
    headingDirectionFootprintInWorldFrame.z() = 0.0;

    try {
      Eigen::Vector3d xAxis = Eigen::Vector3d::UnitX();
      orientationFootprintToWorld.setFromVectors(xAxis, headingDirectionFootprintInWorldFrame.toImplementation());
      // Only update the pose if setFromVectors(..) is successful.
      this->poseFootprintToOdom_.getPosition() = positionWorldToFootprintInWorldFrame;
      this->poseFootprintToOdom_.getRotation() = orientationFootprintToWorld;
    } catch (const std::runtime_error& error) {
      MELO_WARN("Caught kindr setFromVectors() exception when computing footprint orientation: %s", error.what());
    }
  }

  // Find feetcenter orientation.
  kindr::Position3D headingDirectionFeetcenterInWorldFrame;
  getHeadingDirectionFeetcenter(headingDirectionFeetcenterInWorldFrame, model);
  headingDirectionFeetcenterInWorldFrame.z() = 0.0;

  try {
    Eigen::Vector3d xAxis = Eigen::Vector3d::UnitX();
    orientationFeetcenterToWorld.setFromVectors(xAxis, headingDirectionFeetcenterInWorldFrame.toImplementation());
    // Only update the pose if setFromVectors(..) is successful.
    this->poseFeetcenterToOdom_.getPosition() = positionWorldToFeetcenterInWorldFrame;
    this->poseFeetcenterToOdom_.getRotation() = orientationFeetcenterToWorld;
  } catch (const std::runtime_error& error) {
    MELO_WARN("Caught kindr setFromVectors() exception when computing feetcenter orientation: %s", error.what());
  }
}

template <typename ConcreteAnymalDescription_, typename AnymalState_>
void AnymalFramesGenerator<ConcreteAnymalDescription_, AnymalState_>::getHeadingDirectionFootprint(kindr::Position3D& headingInWorldFrame,
                                                                                                   const RobotModel& model) {
  const kindr::Position3D positionForeFeetMidPointInWorldFrame =
      (positionsWorldToFootInWorldFrame_[FootEnum::LF_FOOT] + positionsWorldToFootInWorldFrame_[FootEnum::RF_FOOT]) * 0.5;
  const kindr::Position3D positionHindFeetMidPointInWorldFrame =
      (positionsWorldToFootInWorldFrame_[FootEnum::LH_FOOT] + positionsWorldToFootInWorldFrame_[FootEnum::RH_FOOT]) * 0.5;
  headingInWorldFrame = positionForeFeetMidPointInWorldFrame - positionHindFeetMidPointInWorldFrame;
}

template <typename ConcreteAnymalDescription_, typename AnymalState_>
void AnymalFramesGenerator<ConcreteAnymalDescription_, AnymalState_>::getHeadingDirectionFeetcenter(kindr::Position3D& headingInWorldFrame,
                                                                                                    const RobotModel& model) {
  kindr::Position3D positionWorldToLFFootInWorldFrame;
  model.getPositionWorldToBody(positionWorldToLFFootInWorldFrame.toImplementation(), AD::BodyEnum::LF_FOOT, AD::CoordinateFrameEnum::WORLD);

  kindr::Position3D positionWorldToRFFootInWorldFrame;
  model.getPositionWorldToBody(positionWorldToRFFootInWorldFrame.toImplementation(), AD::BodyEnum::RF_FOOT, AD::CoordinateFrameEnum::WORLD);

  kindr::Position3D positionWorldToLHFootInWorldFrame;
  model.getPositionWorldToBody(positionWorldToLHFootInWorldFrame.toImplementation(), AD::BodyEnum::LH_FOOT, AD::CoordinateFrameEnum::WORLD);

  kindr::Position3D positionWorldToRHFootInWorldFrame;
  model.getPositionWorldToBody(positionWorldToRHFootInWorldFrame.toImplementation(), AD::BodyEnum::RH_FOOT, AD::CoordinateFrameEnum::WORLD);

  const kindr::Position3D positionForeFeetMidPointInWorldFrame =
      (positionWorldToLFFootInWorldFrame + positionWorldToRFFootInWorldFrame) * 0.5;
  const kindr::Position3D positionHindFeetMidPointInWorldFrame =
      (positionWorldToLHFootInWorldFrame + positionWorldToRHFootInWorldFrame) * 0.5;
  headingInWorldFrame = positionForeFeetMidPointInWorldFrame - positionHindFeetMidPointInWorldFrame;
}

}  // namespace anymal_model
