/**
 * @authors     Stephane Caron
 * @affiliation ANYbotics
 * @brief       Implementation of TerrainAdapter
 */

#pragma once

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// robot utils
#include <robot_utils/math/math.hpp>

// loco
#include "loco/common/TerrainModelBase.hpp"
#include "loco/heading_generation/HeadingGenerator.hpp"

namespace loco {

namespace com_support {

struct TerrainAdaptionParams {
  TerrainAdaptionParams() : minHeightScale_(1.0), maxInclinationAngle_(0.0), gravityFactor_(1.0) {}

  bool useTerrainAdaption() const noexcept { return (minHeightScale_ < 1.0 && minHeightScale_ >= 0.0 && maxInclinationAngle_ > 0.0); }

  //! Min value for hright scaling.
  double minHeightScale_;

  //! saturation value for inclination angle.
  double maxInclinationAngle_;

  //! If 1, robot (approximated as inverted pendulum), will align with gravity vector.
  //! If 0, robot will align with local terrain normal.
  //! If value is in (0,1), the projection vector will be linearly interpolated.
  double gravityFactor_;
};

} /* namespace com_support */

/*! Terrain adaption dictates how the torso adapts to terrain variations.
 *
 * This class mainly holds the gravity factor (see TerrainAdaptionParams) and computes the desired height in the terrain-adaptive virtual
 * plane, which is consumed by the foot placement strategy.
 *
 */
class TerrainAdapter {
 public:
  TerrainAdapter() : positionPlaneToDesiredTargetHeightInPlaneFrame_(0.0, 0.0, 0.45) {}

  /*! Add variables to the signal logger.
   *
   * @param ns Signal namespace.
   *
   */
  void addVariablesToLog(const std::string& ns = "/motion_generation_loco/TerrainAdapter") const;

  /*! Load parameters from XML configuration file.
   *
   * @param xmlHandle Handle to XML configuration.
   * @return success True if everything went well.
   *
   */
  bool loadParameters(const TiXmlHandle& xmlHandle);

  /*! Update height in terrain-adaptive frame.
   *
   * @param desiredHeight Desired height above ground on flat terrain.
   * @param orientationPlaneToWorld Orientation of virtual plane frame.
   * @param headingGenerator Heading generator.
   * @param terrain Terrain model.
   * @param isWalking True if the robot is walking or transitioning to walking.
   *
   */
  void update(double desiredHeight, const RotationQuaternion& orientationPlaneToWorld, const loco::HeadingGenerator& headingGenerator,
              const loco::TerrainModelBase& terrain, bool isWalking);

  //! Return desired height above ground.
  const Position& getPositionPlaneToDesiredTargetHeightInPlaneFrame() const { return positionPlaneToDesiredTargetHeightInPlaneFrame_; }

  /*! Get gravity factor setting:
   *
   * - 1: robot (approximated as inverted pendulum) aligns with gravity vector
   * - 0: robot aligns with local terrain normal
   * - between 0 and 1: projection vector is linearly interpolated
   *
   * @return gravityFactor Gravity factor.
   *
   */
  double getGravityFactor() const { return terrainAdaptionParams_.gravityFactor_; }

 protected:
  //! Parameters for terrain adaption.
  com_support::TerrainAdaptionParams terrainAdaptionParams_;

  //! Reference height adapted to terrain
  Position positionPlaneToDesiredTargetHeightInPlaneFrame_;
};

} /* namespace loco */
