/*
 * TerrainPerceptionHorizontalPlane.hpp
 *
 *  Created on: Apr 15, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/TerrainModelHorizontalPlane.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/terrain_perception/TerrainPerceptionBase.hpp"

namespace loco {

class TerrainPerceptionHorizontalPlane : public TerrainPerceptionBase {
 public:
  TerrainPerceptionHorizontalPlane(TerrainModelHorizontalPlane& terrainModel, WholeBody& wholeBody);
  ~TerrainPerceptionHorizontalPlane() override = default;

  bool initialize(double dt) override;

  /*! Advance in time
   * @param dt  time step [s]
   */
  bool advance(double dt) override;

  void updateControlFrameOrigin() override;
  void updateControlFrameAttitude() override;

 protected:
  TerrainModelHorizontalPlane& terrainModel_;
  TorsoBase& torso_;
  Legs& legs_;
};

} /* namespace loco */
