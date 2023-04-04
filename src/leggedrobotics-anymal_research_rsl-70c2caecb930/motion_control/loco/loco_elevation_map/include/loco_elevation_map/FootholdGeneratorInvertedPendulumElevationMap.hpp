/*
 * FootholdGeneratorInvertedPendulum.hpp
 *
 *  Created on: Feb 1, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

//loco
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulum.hpp"

// loco elevation map
#include "loco_elevation_map/TerrainModelElevationMap.hpp"


namespace loco {

class FootholdGeneratorInvertedPendulumElevationMap : public loco::FootholdGeneratorInvertedPendulum {
 public:
  FootholdGeneratorInvertedPendulumElevationMap(
      loco::WholeBody& wholeBody, loco::TerrainModelElevationMap& terrain);
  virtual ~FootholdGeneratorInvertedPendulumElevationMap();

  loco::Position computeWorldToFootholdInWorldFrame(const int legId) override;

  friend std::ostream& operator<< (std::ostream& out, const loco::FootholdGeneratorInvertedPendulumElevationMap& fhGen);

 private:
  inline TerrainModelElevationMap& getTerrainRef();
};


} /* namespace loco */
