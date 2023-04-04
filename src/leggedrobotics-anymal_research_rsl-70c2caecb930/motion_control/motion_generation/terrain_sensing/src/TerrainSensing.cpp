
// terrain sensing.
#include <terrain_sensing/TerrainSensing.hpp>

// tinyxml tools.
#include <tinyxml_tools/tinyxml_tools.hpp>

// message logger.
#include <message_logger/message_logger.hpp>

// tinyxml tools.
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace color = message_logger::color;

namespace loco {
TerrainSensing::TerrainSensing(WholeBody& wholeBody)
    : TerrainModelPlane(),
      wholeBody_(wholeBody),
      gridMap_(),
      mutexGridMap_(),
      freePlaneModel_(),
      mutexFreePlaneModel_(),
      useHeightInformation_(true),
      useGradientInformation_(true),
      distanceFeetOverElevationMap_(0.0) {}

bool TerrainSensing::initialize(double dt) {
  boost::unique_lock<boost::shared_mutex> lock(mutexFreePlaneModel_);
  freePlaneModel_.initialize(dt);
  return true;
}

bool TerrainSensing::loadParameters(const TiXmlHandle& handle) {
  std::cout << color::magenta << "[TerrainSensing] " << color::blue << "Load parameters." << color::def << std::endl;

  TiXmlHandle terrainSensingHandle = handle;
  if (!tinyxml_tools::getChildHandle(terrainSensingHandle, handle, "TerrainSensing")) {
    return false;
  }

  TiXmlHandle feetHandle = handle;
  if (!tinyxml_tools::getChildHandle(feetHandle, terrainSensingHandle, "Feet")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(distanceFeetOverElevationMap_, feetHandle, "foot_center_over_elevation_map")) {
    return false;
  }

  {
    boost::unique_lock<boost::shared_mutex> lock(mutexFreePlaneModel_);
    if (!freePlaneModel_.loadParameters(handle)) {
      return false;
    }
  }

  return true;
}

bool TerrainSensing::getNormal(const Position& positionWorldToLocationInWorldFrame, Vector& normalInWorldFrame) const {
  //  boost::shared_lock<boost::shared_mutex> lock(mutexGridMap_);
  //  if (useHeightInformation_ && useGradientInformation_ && !gridMap_.getLength().isZero()) {
  //    const grid_map::Position location = positionToGridMapPosition(positionWorldToLocationInWorldFrame);
  //    if (gridMap_.isInside(location)) {
  //      grid_map::Index index;
  //      if(gridMap_.getIndex(location, index)) {
  //        if (gridMap_.exists(terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::NormalX]) &&
  //            gridMap_.exists(terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::NormalY]) &&
  //            gridMap_.exists(terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::NormalZ])) {
  //          normalInWorldFrame.x() = gridMap_.at(terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::NormalX], index);
  //          normalInWorldFrame.y() = gridMap_.at(terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::NormalY], index);
  //          normalInWorldFrame.z() = gridMap_.at(terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::NormalZ], index);
  //          return true;
  //        } else {
  //          MELO_WARN_STREAM("[TerrainSensing::getNormal] Normal vector layers missing.");
  //        }
  //      }
  //    }
  //  }

  boost::shared_lock<boost::shared_mutex> lock(mutexFreePlaneModel_);
  return freePlaneModel_.getNormal(positionWorldToLocationInWorldFrame, normalInWorldFrame);
}

bool TerrainSensing::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const {
  double height;
  if (getHeight(positionWorldToLocationInWorldFrame, height)) {
    positionWorldToLocationInWorldFrame.z() = height;
    return true;
  }
  return false;
}

bool TerrainSensing::getHeight(const Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexGridMap_);
    if (useHeightInformation_ && !gridMap_.getLength().isZero()) {
      const grid_map::Position location = positionToGridMapPosition(positionWorldToLocationInWorldFrame);
      if (gridMap_.isInside(location)) {
        grid_map::Index index;
        if (gridMap_.getIndex(location, index)) {
          if (gridMap_.exists(terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::Height]) &&
              gridMap_.isValid(index, terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::Height])) {
            heightInWorldFrame = gridMap_.at(terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::Height], index);
          }
          return true;
        }
      }
    }
  }

  {
    boost::shared_lock<boost::shared_mutex> lock(mutexFreePlaneModel_);
    return freePlaneModel_.getHeight(positionWorldToLocationInWorldFrame, heightInWorldFrame);
  }
}

bool TerrainSensing::getMaxHeightBetweenTwoPoints(const Position& point1, const Position& point2, double& height) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexGridMap_);
  height = std::fmax(point1.z(), point2.z());

  if (useHeightInformation_ && !gridMap_.getLength().isZero()) {
    const grid_map::Position initialPoint = positionToGridMapPosition(point1);
    const grid_map::Position finalPoint = positionToGridMapPosition(point2);
    if (gridMap_.isInside(initialPoint) && gridMap_.isInside(finalPoint) &&
        gridMap_.exists(terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::Height])) {
      for (grid_map::LineIterator iterator(gridMap_, initialPoint, finalPoint); !iterator.isPastEnd(); ++iterator) {
        if (gridMap_.isValid(*iterator, terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::Height])) {
          const double currentHeight = gridMap_.at(terrain_sensing::GridLayersMap[terrain_sensing::GridLayers::Height], *iterator);
          if (currentHeight > height) {
            height = currentHeight;
          }
        }
      }
    }
  }

  return true;
}

bool TerrainSensing::setMap(grid_map::GridMap& elevationMap) {
  boost::unique_lock<boost::shared_mutex> lockGridMap(mutexGridMap_);

  // Copy new grid map.
  gridMap_ = elevationMap;

  /*
   * Delete layers that, we don't need.
   * Note: Objectives and constraints that depend on these layers will be automatically disabled without
   * displaying any notification.
   */
  const auto layers = gridMap_.getLayers();
  for (const auto& layer : layers) {
    bool deleteLayer = true;

    if (useHeightInformation_ && useGradientInformation_) {
      for (const auto& usedLayer : terrain_sensing::layersXYZ) {
        if (layer == terrain_sensing::GridLayersMap[usedLayer]) {
          deleteLayer = false;
        }
      }
    }

    else if (useHeightInformation_ && !useGradientInformation_) {
      for (const auto& usedLayer : terrain_sensing::layersZ) {
        if (layer == terrain_sensing::GridLayersMap[usedLayer]) {
          deleteLayer = false;
        }
      }
    }

    else if (!useHeightInformation_ && useGradientInformation_) {
      for (const auto& usedLayer : terrain_sensing::layersXY) {
        if (layer == terrain_sensing::GridLayersMap[usedLayer]) {
          deleteLayer = false;
        }
      }
    }

    if (deleteLayer && gridMap_.exists(layer)) {
      if (!gridMap_.erase(layer)) {
        MELO_WARN_STREAM("[TerrainSensing::setMap] Failed to delete layer " << layer << ".");
        return false;
      }
    }
  }

  return true;
}

const grid_map::GridMap& TerrainSensing::getMap() const {
  boost::shared_lock<boost::shared_mutex> lock(mutexGridMap_);
  return gridMap_;
}

const TerrainModelFreePlane& TerrainSensing::getFreePlaneModel() const {
  boost::shared_lock<boost::shared_mutex> lock(mutexFreePlaneModel_);
  return freePlaneModel_;
}

void TerrainSensing::setNormalandPositionInWorldFrame(const Vector& normal, const Position& position) {
  boost::unique_lock<boost::shared_mutex> lock(mutexFreePlaneModel_);
  freePlaneModel_.setNormalandPositionInWorldFrame(normal, position);
}

bool TerrainSensing::getFrictionCoefficientForFoot(const Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexFreePlaneModel_);
  return freePlaneModel_.getFrictionCoefficientForFoot(positionWorldToLocationInWorldFrame, frictionCoefficient);
}

Position TerrainSensing::getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(
    const Position& positionWorldToFootholdInWorldFrame) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexFreePlaneModel_);
  return freePlaneModel_.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(positionWorldToFootholdInWorldFrame);
}

double TerrainSensing::getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(const Position& positionInWorldFrame) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexFreePlaneModel_);
  return freePlaneModel_.getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(positionInWorldFrame);
}

double TerrainSensing::getHeightAboveTerrainAlongSurfaceNormal(const Position& positionWorldToLocationInWorldFrame) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexFreePlaneModel_);
  return freePlaneModel_.getHeightAboveTerrainAlongSurfaceNormal(positionWorldToLocationInWorldFrame);
}

grid_map::Position TerrainSensing::positionToGridMapPosition(const Position& pos) const {
  return {pos.x(), pos.y()};
}

void TerrainSensing::setUseHeightInformation(bool useHeightInformation) noexcept {
  useHeightInformation_ = useHeightInformation;
}

bool TerrainSensing::getUseHeightInformation() const noexcept {
  return useHeightInformation_;
}

void TerrainSensing::setUseGradientInformation(bool useGradientInformation) noexcept {
  useGradientInformation_ = useGradientInformation;
}

bool TerrainSensing::getUseGradientInformation() const noexcept {
  return useGradientInformation_;
}

bool TerrainSensing::getElevationMapValue(double& value, const terrain_sensing::GridLayers& layer,
                                          const Position& positionWorldToLocationInWorldFrame) const noexcept {
  boost::shared_lock<boost::shared_mutex> lock(mutexGridMap_);
  const std::string layerName = terrain_sensing::GridLayersMap[layer];

  if (gridMap_.exists(layerName) && !gridMap_.getLength().isZero()) {
    grid_map::Index index;
    const grid_map::Position location = positionToGridMapPosition(positionWorldToLocationInWorldFrame);
    if (gridMap_.isInside(location) && gridMap_.getIndex(location, index)) {
      if (gridMap_.isValid(index, layerName)) {
        value = gridMap_.at(layerName, index);
        return true;
      }
    }
  }

  MELO_WARN_THROTTLE_STREAM(1.0, "[TerrainSensing::getElevationMapValue] Value not found for layer " << layerName << ".");
  return false;
}

double TerrainSensing::getDistanceFeetOverElevationMap() const noexcept {
  return distanceFeetOverElevationMap_;
}

}  // namespace loco
