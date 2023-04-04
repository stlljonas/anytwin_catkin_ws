/**
 * @authors     Stephane Caron
 * @affiliation ANYbotics
 * @brief       Implementation of TerrainAdapter
 */

#include <loco/torso_control/TerrainAdapter.hpp>

namespace loco {

bool TerrainAdapter::loadParameters(const TiXmlHandle& xmlHandle) {
  MELO_DEBUG_STREAM(message_logger::color::magenta << "[TerrainAdapter] " << message_logger::color::blue << "Load parameters."
                                                   << message_logger::color::def)
  TiXmlHandle terrainHandle = xmlHandle;
  if (!tinyxml_tools::getChildHandle(terrainHandle, xmlHandle, "TerrainAdaption")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(terrainAdaptionParams_.minHeightScale_, terrainHandle, "min_height_scale")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(terrainAdaptionParams_.maxInclinationAngle_, terrainHandle, "max_interp_angle_deg")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(terrainAdaptionParams_.gravityFactor_, terrainHandle, "gravity_factor")) {
    return false;
  }
  terrainAdaptionParams_.maxInclinationAngle_ *= 0.017453293;  // degree to rad.
  robot_utils::boundToRange(&terrainAdaptionParams_.gravityFactor_, 0.0, 1.0);
  return true;
}

void TerrainAdapter::update(double desiredHeight, const RotationQuaternion& orientationPlaneToWorld,
                            const loco::HeadingGenerator& headingGenerator, const loco::TerrainModelBase& terrain, bool isWalking) {
  // Terrain scaling factor for desired height (heuristic, helps to satisfy kinematic constraints).
  double terrainScaling = 1.0;
  if (terrainAdaptionParams_.useTerrainAdaption()) {
    // Compute terrain orientation w.r.t. to robot heading direction.
    Position positionWorldToFootprintCenterInWorldFrame;
    headingGenerator.computeCurrentFootPrintCenterInWorldFrame(positionWorldToFootprintCenterInWorldFrame);

    Vector headingInWorldFrame;
    headingGenerator.getTorsoHeadingDirectionInWorldFrame(headingInWorldFrame);
    headingInWorldFrame.z() = 0.0;

    double terrainPitch, terrainRoll;
    terrain.getTerrainOrientation(positionWorldToFootprintCenterInWorldFrame, headingInWorldFrame, terrainPitch, terrainRoll);

    const double angleMerged =
        std::fmin(0.8 * std::fabs(terrainPitch) + 0.2 * std::fabs(terrainRoll), terrainAdaptionParams_.maxInclinationAngle_);

    terrainScaling = robot_utils::linearlyInterpolate(1.0, terrainAdaptionParams_.minHeightScale_, 0.0,
                                                      terrainAdaptionParams_.maxInclinationAngle_, angleMerged);
  }

  // Compute position plane to desired height.
  const Vector worldZNormalInWorldFrame = Vector::UnitZ();
  const Vector planeNormalInWorldFrame = orientationPlaneToWorld.rotate(worldZNormalInWorldFrame);
  const Vector mergedPlaneNormalInWorldFrame = terrainAdaptionParams_.gravityFactor_ * worldZNormalInWorldFrame +
                                               (1.0 - terrainAdaptionParams_.gravityFactor_) * planeNormalInWorldFrame;

  positionPlaneToDesiredTargetHeightInPlaneFrame_ =
      static_cast<Position>(orientationPlaneToWorld.inverseRotate(mergedPlaneNormalInWorldFrame * desiredHeight * terrainScaling));

  if (isWalking) {
    positionPlaneToDesiredTargetHeightInPlaneFrame_.toImplementation().head<2>().setZero();
  }
}

void TerrainAdapter::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(positionPlaneToDesiredTargetHeightInPlaneFrame_, "positionPlaneToDesiredTargetHeightInPlaneFrame", ns, "m");
}

} /* namespace loco */