/*
 * terrain_sensing.hpp
 *
 *  Created on: June 12, 2019
 *      Author: Fabian Jenelten
 */

// std_utils
#include <std_utils/std_utils.hpp>

#pragma once

namespace loco {
namespace terrain_sensing {

// Grid map layers.
CONSECUTIVE_ENUM(GridLayers, Height, FootholdScore, SmoothElevation)
static std::map<GridLayers, std::string> GridLayersMap = {{GridLayers::Height, "elevation_filled"},
                                                          {GridLayers::FootholdScore, "foothold_score"},
                                                          {GridLayers::SmoothElevation, "elevation_smooth"},
                                                          {GridLayers::SIZE, "SIZE"}};

constexpr std::array<GridLayers, 3> layersXYZ = {GridLayers::SmoothElevation, GridLayers::Height, GridLayers::FootholdScore};
constexpr std::array<GridLayers, 2> layersXY = {GridLayers::Height, GridLayers::FootholdScore};
constexpr std::array<GridLayers, 2> layersZ = {GridLayers::SmoothElevation, GridLayers::Height};

}  // namespace terrain_sensing
}  // namespace loco
