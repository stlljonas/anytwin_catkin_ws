/*
 * TerrainPerceptionFreePlaneAdapted.hpp
 *
 *  Created on: Jul 11, 2017
 *      Author: dbellicoso
 */

#pragma once

#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"

namespace loco {

class TerrainPerceptionFreePlaneAdapted : public loco::TerrainPerceptionFreePlane {
 private:
  using Base = loco::TerrainPerceptionFreePlane;

 public:
  TerrainPerceptionFreePlaneAdapted(
      TerrainModelPlane& terrainModel, WholeBody& wholeBody, HeadingGenerator& headingGenerator,
      TerrainPerceptionFreePlane::EstimatePlaneInFrame estimatePlaneInFrame = TerrainPerceptionFreePlane::EstimatePlaneInFrame::World,
      ControlFrameHeading referenceHeading = ControlFrameHeading::Hips);
  ~TerrainPerceptionFreePlaneAdapted() override = default;

 protected:
  void updatePlaneEstimation() override;
};

} /* namespace loco */
