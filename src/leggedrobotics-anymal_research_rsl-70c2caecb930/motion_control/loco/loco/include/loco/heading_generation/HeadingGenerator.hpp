/*
 * HeadingGenerator.hpp
 *
 *  Created on: Jan 26, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/WholeBody.hpp>
#include <loco/common/typedefs.hpp>

namespace loco {

/*! Heading generator.
 *
 * This module keeps track of foot contacts to compute the torso heading and foot-center frames. Each frame is available in three flavors:
 * - Last: computed from the last effective contact of each leg.
 * - Current: computed from the current configuration of each leg, in contact or not.
 * - Planned: computed from the desired position of each foot, where next footholds are stored for swing legs.
 */
class HeadingGenerator {
 public:
  HeadingGenerator() = default;
  virtual ~HeadingGenerator() = default;

  virtual void getTorsoHeadingDirectionInWorldFrame(Vector& headingDirectionInWorldFrame) const = 0;
  virtual void getLegsHeadingDirectionFromCurrentFeetInWorldFrame(Vector& headingDirectionInWorldFrame) const = 0;
  virtual void getLegsHeadingDirectionFromLastContactsInWorldFrame(loco::Vector& headingDirectionInWorldFrame) const = 0;
  virtual void getLegsHeadingDirectionFromPlannedContactsInWorldFrame(loco::Vector& headingDirectionInWorldFrame) const = 0;

  virtual bool computeLastFootPrintCenterInWorldFrame(Position& footPrintCenterInWorldFrame) const = 0;
  virtual bool computeCurrentFootPrintCenterInWorldFrame(Position& footPrintCenterInWorldFrame) const = 0;
  virtual bool computePlannedFootPrintCenterInWorldFrame(Position& footPrintCenterInWorldFrame) const = 0;

  virtual void getOrientationWorldToTorsoHeading(RotationQuaternion& orientationWorldToHeading) const;
  virtual void getOrientationWorldToPreviousFootprintHeading(RotationQuaternion& orientationWorldToHeading) const;
  virtual void getOrientationWorldToCurrentFootprintHeading(RotationQuaternion& orientationWorldToHeading) const;
  virtual void getOrientationWorldToPlannedFootprintHeading(RotationQuaternion& orientationWorldToHeading) const;

  /*! Get the homogeneous transform of the last FootPrintCenter frame, computed from the last effective contact of each leg.
   *
   * @note This frame is the one labeled "footprint" in the ROS TF tree.
   *
   * @return Homogeneous transform @c poseLastFootPrintCenterToWorld.
   */
  loco::Pose getPoseLastFootPrintCenterToWorld() const;
};

} /* namespace loco */
