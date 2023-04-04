/*
 * VirtualPlaneFrame.hpp
 *
 *  Created on: June 01, 2017
 *      Author: Fabian Jenelten, Dario Bellicoso
 */

// zmp optimizer
#include "zmp_optimizer/VirtualPlaneFrame.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace zmp {

VirtualPlaneFrame::VirtualPlaneFrame(vpf::VirtualPlaneFrameEnum planeFrame) : VirtualPlaneFrameBase(), frameType_(planeFrame) {
  planeNormalInPlaneFrame_ = motion_generation::Vector::UnitZ();
}

void VirtualPlaneFrame::setVirtualPlaneFrameEnum(vpf::VirtualPlaneFrameEnum planeFrame) {
  frameType_ = planeFrame;
}

bool VirtualPlaneFrame::computeVirtualPlaneFrame(const loco::HeadingGenerator& headingGenerator, const loco::WholeBody& wholeBody,
                                                 const loco::TerrainModelBase& terrain,
                                                 const loco::ContactScheduleZmp& /*contactSchedule*/) {
  /*
   * Virtual plane frame:
   *  > The center is located at the footprint center, projected onto the local terrain.
   *  > The yaw of the frame is equal to the yaw of the control (or footprint) frame while
   *    roll and pitch angle are taken from the terrain.
   * Virtual plane:
   *  > x-y-plane of the the virtual plane frame.
   *  > plane normal corresponds to the surface normal of the local terrain
   */

  // Compute footprint center in world frame.
  loco::Position positionWorldToFootPrintCenterInWorldFrame;
  if (!headingGenerator.computeCurrentFootPrintCenterInWorldFrame(positionWorldToFootPrintCenterInWorldFrame)) {
    return false;
  }

  // Center of plane frame: project footprint center onto terrain surface along local terrain normal.
  poseVirtualPlaneToWorld_.getPosition() =
      terrain.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(positionWorldToFootPrintCenterInWorldFrame);

  // Get plane normal at footprint center.
  terrain.getNormal(poseVirtualPlaneToWorld_.getPosition(), planeNormalInWorldFrame_);

  // Compute orientation.
  loco::RotationQuaternion orientationWorldToVirtualPlane;
  switch (frameType_) {
    case vpf::VirtualPlaneFrameEnum::alignedWithRefVel: {
      Eigen::Vector3d virtualPlaneHeadingInControlFrame;

      // If there exists a non-trivial reference velocity --> align with it.
      const auto& desiredLinearVelocity = wholeBody.getTorso().getDesiredState().getLinearVelocityTargetInControlFrame();
      if (desiredLinearVelocity.toImplementation().norm() > 1e-7) {
        virtualPlaneHeadingInControlFrame = Eigen::Vector3d(desiredLinearVelocity.x(), desiredLinearVelocity.y(), 0.0);

        // Make sure VPF is aligned with Control frame (that easier when optimizing for orientations).
        if (virtualPlaneHeadingInControlFrame.x() < 0.0) {
          const auto& virtualPlaneHeadingInControlFrameOld = virtualPlaneHeadingInControlFrame;
          virtualPlaneHeadingInControlFrame = -1.0 * virtualPlaneHeadingInControlFrameOld;
        }

      } else {  // Otherwise --> use control frame.
        orientationWorldToVirtualPlane = wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();
        break;
      }

      // Compute control to virtual plane orientation.
      loco::RotationQuaternion orientationControlToVirtualPlane;
      try {
        Eigen::Vector3d controlHeadingInControlFrame = Eigen::Vector3d::UnitX();
        orientationControlToVirtualPlane.setFromVectors<Eigen::Vector3d>(virtualPlaneHeadingInControlFrame, controlHeadingInControlFrame);
      } catch (const std::runtime_error& error) {
        MELO_WARN_STREAM(error.what() << "setFromVectors in VirtualPlaneFrame::computeVirtualPlaneFrame()." << std::endl)
        orientationControlToVirtualPlane.setIdentity();
      }

      // Compute world to virtual plane orientation.
      orientationWorldToVirtualPlane =
          orientationControlToVirtualPlane * wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();
    } break;

    case vpf::VirtualPlaneFrameEnum::controlFrame: {
      orientationWorldToVirtualPlane = wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();
    } break;

    case vpf::VirtualPlaneFrameEnum::worldFrame: {
      orientationWorldToVirtualPlane.setIdentity();
    } break;

    case vpf::VirtualPlaneFrameEnum::actualTorso: {
      loco::RotationQuaternion orientatioTerrainSurfaceToVirtualPlane;
      headingGenerator.getOrientationWorldToTorsoHeading(orientatioTerrainSurfaceToVirtualPlane);
      const loco::RotationQuaternion orientationWorldToTerrainSurface =
          computeOrientationTerrainSurfaceToWorld(poseVirtualPlaneToWorld_.getPosition()).invert();
      orientationWorldToVirtualPlane = orientatioTerrainSurfaceToVirtualPlane * orientationWorldToTerrainSurface;
    } break;

    case vpf::VirtualPlaneFrameEnum::previousFootprint: {
      loco::RotationQuaternion orientatioTerrainSurfaceToVirtualPlane;
      headingGenerator.getOrientationWorldToPreviousFootprintHeading(orientatioTerrainSurfaceToVirtualPlane);
      const loco::RotationQuaternion orientationWorldToTerrainSurface =
          computeOrientationTerrainSurfaceToWorld(poseVirtualPlaneToWorld_.getPosition()).invert();
      orientationWorldToVirtualPlane = orientatioTerrainSurfaceToVirtualPlane * orientationWorldToTerrainSurface;
    } break;

    case vpf::VirtualPlaneFrameEnum::actualFootprint: {
      loco::RotationQuaternion orientatioTerrainSurfaceToVirtualPlane;
      headingGenerator.getOrientationWorldToCurrentFootprintHeading(orientatioTerrainSurfaceToVirtualPlane);
      const loco::RotationQuaternion orientationWorldToTerrainSurface =
          computeOrientationTerrainSurfaceToWorld(poseVirtualPlaneToWorld_.getPosition()).invert();
      orientationWorldToVirtualPlane = orientatioTerrainSurfaceToVirtualPlane * orientationWorldToTerrainSurface;
    } break;

    default: {
      MELO_WARN_STREAM("[VirtualPlaneFrame::computeVirtualPlaneFrame] Unhandled heading frame!.")
      poseVirtualPlaneToWorld_.getRotation().setIdentity();
      return false;
    }
  }

  // Update pose.
  poseVirtualPlaneToWorld_.getRotation() = orientationWorldToVirtualPlane.inverted();
  planeNormalInPlaneFrame_ = poseVirtualPlaneToWorld_.getRotation().inverseRotate(planeNormalInWorldFrame_);

  return true;
}

void VirtualPlaneFrame::computeLocalTerrainOrientationInWorldFrame(double& terrainPitch, double& terrainRoll) const {
  terrainPitch = std::atan2(planeNormalInWorldFrame_.x(), planeNormalInWorldFrame_.z());
  terrainRoll = std::atan2(planeNormalInWorldFrame_.y(), planeNormalInWorldFrame_.z());
}

loco::RotationQuaternion VirtualPlaneFrame::computeOrientationTerrainSurfaceToWorld(
    const loco::Position& /*positionWorldToLocationInWorldFrame*/) const {
  double terrainPitch;
  double terrainRoll;
  computeLocalTerrainOrientationInWorldFrame(terrainPitch, terrainRoll);

  return loco::RotationQuaternion(loco::AngleAxis(terrainRoll, -1.0, 0.0, 0.0)) *
         loco::RotationQuaternion(loco::AngleAxis(terrainPitch, 0.0, 1.0, 0.0));
}

void VirtualPlaneFrame::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(frameType_, "frameType", ns, "m");
  signal_logger::add(planeNormalInPlaneFrame_, "planeNormalInPlaneFrame", ns);
  signal_logger::add(planeNormalInWorldFrame_, "planeNormalInWorldFrame", ns);
  signal_logger::add(poseVirtualPlaneToWorld_, "poseVirtualPlaneToWorld", ns);
}

} /* namespace zmp */
