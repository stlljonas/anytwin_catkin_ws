/*
 * VirtualPlaneFrame.hpp
 *
 *  Created on: June 01, 2017
 *      Author: Fabian Jenelten, Dario Bellicoso
 */

#pragma once

// motion generation utils
#include "motion_generation_utils/VirtualPlaneFrameBase.hpp"

// motion generation
#include "motion_generation/ContactScheduleZmp.hpp"

// loco
#include <loco/common/WholeBody.hpp>
#include "loco/common/TerrainModelBase.hpp"
#include "loco/heading_generation/HeadingGenerator.hpp"

namespace zmp {

namespace vpf {

// plane frame type
enum class VirtualPlaneFrameEnum : unsigned int {
  worldFrame,
  controlFrame,
  actualTorso,
  previousFootprint,
  actualFootprint,
  alignedWithRefVel
};

} /* namespace vpf */

/*! * Virtual plane frame, the x-axis of which is aligned with the desired velocity vector.
 *
 */
class VirtualPlaneFrame : public motion_generation::VirtualPlaneFrameBase {
 public:
  explicit VirtualPlaneFrame(vpf::VirtualPlaneFrameEnum planeFrame = vpf::VirtualPlaneFrameEnum::actualTorso);
  ~VirtualPlaneFrame() override = default;

  //! Set plane frame type.
  void setVirtualPlaneFrameEnum(vpf::VirtualPlaneFrameEnum planeFrame);

  //! Update components of the virtual plane frame.
  bool computeVirtualPlaneFrame(const loco::HeadingGenerator& headingGenerator, const loco::WholeBody& wholeBody,
                                const loco::TerrainModelBase& terrain, const loco::ContactScheduleZmp& contactSchedule);

  /*! Add variables to the signal logger.
   *
   * @param ns Namespace prefix.
   */
  void addVariablesToLog(const std::string& ns = "/zmp_optimizer/VirtualPlaneFrame") const;

 protected:
  void computeLocalTerrainOrientationInWorldFrame(double& terrainPitch, double& terrainRoll) const;

  loco::RotationQuaternion computeOrientationTerrainSurfaceToWorld(const loco::Position& positionWorldToLocationInWorldFrame) const;

  //! Type of the virtual plane frame
  vpf::VirtualPlaneFrameEnum frameType_;
};

} /* namespace zmp */
