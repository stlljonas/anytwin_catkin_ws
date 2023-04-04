/*
 * ElevationMapUser.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: Péter Fankhauser
 */

#include <locomotion_planner/common/ElevationMapUser.hpp>

#include <message_logger/message_logger.hpp>

namespace locomotion_planner {

ElevationMapUser::ElevationMapUser(Parameters& parameters)
    : isValid_(false),
      parameters_(parameters)
{
}

ElevationMapUser::~ElevationMapUser()
{
}

std::mutex& ElevationMapUser::getMutex()
{
  return mutex_;
}

void ElevationMapUser::setMap(const grid_map::GridMap& map)
{
  elevationMap_ = map;
  signedDistanceField_.clear();
  if(map.getFrameId().empty()) {
    MELO_WARN("Elevation map received is not valid, due to empty reference frame ID");
    isValid_ = false;
    return;
  }
  isValid_ = true;
}

const grid_map::GridMap& ElevationMapUser::getMap()
{
  return elevationMap_;
}

void ElevationMapUser::clearMap()
{
  isValid_ = false;
  signedDistanceField_.clear();
}

bool ElevationMapUser::isMapValid()
{
  return isValid_;
}

void ElevationMapUser::computeSignedDistanceField(const std::string& layer)
{
  MELO_DEBUG_STREAM("Computing SDF for layer " << layer << ".");
  if (!isValid_) return;
  if (!(signedDistanceField_.find(layer) == signedDistanceField_.end())) return; // Already computed.
  signedDistanceField_[layer].calculateSignedDistanceField(elevationMap_, layer, 0.3);
}

const grid_map::SignedDistanceField& ElevationMapUser::getSignedDistanceField(const std::string& layer) const
{
  return signedDistanceField_.at(layer);
}

bool ElevationMapUser::getSurfaceNormalForPosition(const Position& position, Vector& surfaceNormal)
{
  if (!isValid_) return false;
  grid_map::Index index;
  elevationMap_.getIndex(position.vector().head(2), index);
  if (!elevationMap_.isValid(index)) return false;
  surfaceNormal.x() = elevationMap_.at("normal_vectors_x", index);
  surfaceNormal.y() = elevationMap_.at("normal_vectors_y", index);
  surfaceNormal.z() = elevationMap_.at("normal_vectors_z", index);
  return true;
}

bool ElevationMapUser::findClosestValidFoothold(Position& position, std::vector<Position>& candidatePositions)
{
  grid_map::SpiralIterator iterator(elevationMap_, position.vector().head(2), parameters_.getFootholdSearchAreaSize() / 2.0);
  return findClosestValidFoothold(iterator, position, candidatePositions);
}

bool ElevationMapUser::findClosestValidFoothold(grid_map::SpiralIterator& iterator, Position& position,
                                                std::vector<Position>& candidatePositions)
{
  if (!isValid_) return false;
  const grid_map::Matrix& elevationData(elevationMap_[parameters_.getElevationLayer()]);
  const grid_map::Matrix& footholdBinaryData(elevationMap_[parameters_.getFootholdScoreLayer()]);

  // Iterate in spiral around nominal foothold
  for (; !iterator.isPastEnd(); ++iterator) {

    grid_map::Position candidatePosition;
    float sumOfHeights = 0;
    size_t nCells = 0;
    if (!elevationMap_.getPosition(*iterator, candidatePosition)) return false;
    bool candidate = true;
    // Check that foothold area around iterator position is valid
    for (grid_map::CircleIterator footIterator(elevationMap_, candidatePosition, parameters_.getFootholdSize() / 2.0);
         !footIterator.isPastEnd(); ++footIterator) {
      // Iterate through cells of candidate foothold area.
      const grid_map::Index index(*footIterator);
      if (footholdBinaryData(index(0), index(1)) < 0.5 || !std::isfinite(footholdBinaryData(index(0), index(1)))) {
        candidate = false;
        break;
      }
      sumOfHeights += elevationData(index(0), index(1));
      ++nCells;
    }

    position.vector().head(2) = candidatePosition;
    if (candidate) {
      position.z() = sumOfHeights / (float) nCells + parameters_.getFootCenterHeight();
      return true;
    } else {
      const grid_map::Index index(*iterator);
      const float height = elevationData(index(0), index(1));
      if (std::isfinite(height)) position.z() = height + parameters_.getFootCenterHeight();
      candidatePositions.push_back(position);
    }
  }
  return false;
}

}
