/*
 * ElevationMapUser.hpp
 *
 *  Created on: Sep 7, 2017
 *      Author: Péter Fankhauser
 */

#pragma once

#include "locomotion_planner/common/type_defs.hpp"
#include "locomotion_planner/common/Parameters.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

#include <mutex>

namespace locomotion_planner {

class ElevationMapUser
{
 public:
  ElevationMapUser(Parameters& parameters);
  virtual ~ElevationMapUser();

  std::mutex& getMutex();
  void setMap(const grid_map::GridMap& map);
  const grid_map::GridMap& getMap();
  void clearMap();
  bool isMapValid();
  void computeSignedDistanceField(const std::string& layer);
  const grid_map::SignedDistanceField& getSignedDistanceField(const std::string& layer) const;
  bool getSurfaceNormalForPosition(const Position& position, Vector& surfaceNormal);
  bool findClosestValidFoothold(Position& position, std::vector<Position>& candidatePositions);
  bool findClosestValidFoothold(grid_map::SpiralIterator& iterator, Position& position, std::vector<Position>& candidatePositions);

 private:
  Parameters& parameters_;
  grid_map::GridMap elevationMap_;
  std::unordered_map<std::string, grid_map::SignedDistanceField> signedDistanceField_;
  std::mutex mutex_;
  bool isValid_;
};

}
