/*
 * TerrainModelElevationMap.cpp
 *
 *  Created on: Feb 28, 2017
 *      Author: Tanja Baumann, Dario Bellicoso
 */

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// loco elevation map
#include "loco_elevation_map/TerrainModelElevationMap.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

namespace loco {

TerrainModelElevationMap::TerrainModelElevationMap()
    : TerrainModelPlane(),
      freePlaneModel_(),
      commandVelocity_(),
      footholdSearchRadius_(0.15),
      circleDistancePolygon_(0.05),
      circleRadiusPolygon_(0.03),
      heightLayerName_("elevation_inpaint"),
      mapUnprocessedAvailable_(false),
      mapProcessedAvailable_(false),
      workerTimestep_(0.1),
      workerPriority_(0),
      invertedPendulumFootholds_(4, loco::Position()),
      binaryFootholds_(4, loco::Position()),
      polygonFootholds_(4, loco::Position())
{

}

bool TerrainModelElevationMap::initialize(double dt) {
  freePlaneModel_.initialize(dt);

  worker_.reset(
      new any_worker::Worker("line_map_worker", workerTimestep_,
                             std::bind(&TerrainModelElevationMap::lineMapProcessing, this)));
//  worker_.reset(new any_worker::Worker("polygon_map_worker", workerTimestep_, std::bind(&TerrainModelElevationMap::polygonMapProcessing, this)));
  worker_->start(workerPriority_);
  return true;
}

bool TerrainModelElevationMap::loadParameters(const TiXmlHandle& handle) {
  if (!freePlaneModel_.loadParameters(handle)) { return false; }

  TiXmlHandle terrainModelHandle = handle;
  if (!tinyxml_tools::getChildHandle(terrainModelHandle, handle, "TerrainModel")) { return false; }

  TiXmlHandle parametersHandle = handle;
  if (!tinyxml_tools::getChildHandle(parametersHandle, terrainModelHandle, "Parameters")) { return false; }

  if (!tinyxml_tools::loadParameter(circleDistancePolygon_, parametersHandle, "circleDistancePolygon")) { return false;}
  if (!tinyxml_tools::loadParameter(circleRadiusPolygon_, parametersHandle, "circleRadiusPolygon")) { return false;}
  if (!tinyxml_tools::loadParameter(footholdSearchRadius_, parametersHandle, "footholdSearchRadius")) { return false;}
  if (!tinyxml_tools::loadParameter(workerPriority_, parametersHandle, "workerPriority")) { return false;}
  if (!tinyxml_tools::loadParameter(workerTimestep_, parametersHandle, "workerTimestep")) { return false;}

  return true;
}

void TerrainModelElevationMap::setNormalandPositionInWorldFrame(
    const loco::Vector& normal, const loco::Position& position) {
  freePlaneModel_.setNormalandPositionInWorldFrame(normal, position);
}

void TerrainModelElevationMap::setCommandVelocity(const LinearVelocity& commandVelocity) {
  boost::unique_lock<boost::shared_mutex> lockCommandVelocity(mutexCommandVelocity_);
  commandVelocity_ = commandVelocity;
}

void TerrainModelElevationMap::setMap(const grid_map::GridMap& map) {
  boost::unique_lock<boost::shared_mutex> lockUnprocessedMap(mutexUnprocessedMap_);
  mapUnprocessed_ = map;
  mapUnprocessedAvailable_ = true;
}

bool TerrainModelElevationMap::polygonMapProcessing() {
  if (mapUnprocessedAvailable_) {

//    //Timing
//    robot_utils::HighResolutionClockTimer timer;
//    timer.pinTime("polygonMapProcessing");

    //copy Unprocessed map
    boost::unique_lock<boost::shared_mutex> lockUnprocessedMap(mutexUnprocessedMap_);
    grid_map::GridMap localMapUnprocessed = mapUnprocessed_;
    lockUnprocessedMap.unlock();

    grid_map::GridMap localMapProcessed = localMapUnprocessed;
    localMapProcessed.add("footscore_polygon");

    //copy command velocity
    boost::unique_lock<boost::shared_mutex> lockCommandVelocity(mutexCommandVelocity_);
    LinearVelocity commandVelocityVector(commandVelocity_);
    lockCommandVelocity.unlock();

    //calculate Polygon footscores
    if (commandVelocityVector.vector().norm() != 0) {
      commandVelocityVector.vector().normalize();
    }

    // First iteraton through whole map.
    for (grid_map::GridMapIterator iterator(localMapUnprocessed); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      grid_map::Position candidateFoothold;
      localMapUnprocessed.getPosition(index, candidateFoothold);
      bool isGood = true;

      grid_map::Position secondCirclePoint(candidateFoothold.x() + circleDistancePolygon_ * commandVelocityVector.x(),
                                           candidateFoothold.y() + circleDistancePolygon_ * commandVelocityVector.y());
      grid_map::Polygon polygon;
      if (candidateFoothold == secondCirclePoint) {
        polygon = grid_map::Polygon::fromCircle(candidateFoothold, circleRadiusPolygon_);
      } else {
        polygon = grid_map::Polygon::convexHullOfTwoCircles(candidateFoothold, secondCirclePoint, circleRadiusPolygon_);
      }

      grid_map::Position tempPosition;

      //second iteration over map values in polygon
      for (grid_map::PolygonIterator iterator(localMapUnprocessed, polygon); !iterator.isPastEnd(); ++iterator) {

        const grid_map::Index indexPolygon(*iterator);
        bool tempPositionWithinMap = localMapUnprocessed.getPosition(indexPolygon, tempPosition);

        if (tempPositionWithinMap) {
          if (localMapUnprocessed.isValid(indexPolygon, "footscore_binary")
              && localMapUnprocessed.at("footscore_binary", indexPolygon) == 0.0) {
            isGood = false;
          }
        }
      }

      if (isGood) {
        localMapProcessed.at("footscore_polygon", index) = 1.0;
      } else {
        localMapProcessed.at("footscore_polygon", index) = 0.0;
      }
    }

    {
      //copy local processed map into mapProcessed_
      boost::unique_lock<boost::shared_mutex> lockProcessedMap(mutexProcessedMap_);
      mapProcessed_ = localMapProcessed;
      mapProcessedAvailable_ = true;
    }

//    //Timing
//    timer.splitTime("polygonMapProcessing");
//    std::cout << timer << std::endl;
//    std::cout << "averageTime: "<<timer.getAverageElapsedTimeUSec("polygonMapProcessing")<< std::endl;
//    double timeSum = nMeasurements_*averageTime_;
//    timeSum +=timer.getAverageElapsedTimeUSec("polygonMapProcessing");
//    nMeasurements_++;
//    averageTime_= timeSum/nMeasurements_;
//    std::cout<<"averageTime: "<<averageTime_<<std::endl;
  }

  return true;
}

bool TerrainModelElevationMap::lineMapProcessing() {
  if (mapUnprocessedAvailable_) {

//    //Timing
//    robot_utils::HighResolutionClockTimer timer;
//    timer.pinTime("polygonMapProcessing");

    //copy Unprocessed map
    boost::unique_lock<boost::shared_mutex> lockUnprocessedMap(mutexUnprocessedMap_);
    grid_map::GridMap localMapUnprocessed = mapUnprocessed_;
    lockUnprocessedMap.unlock();

    grid_map::GridMap localMapProcessed = localMapUnprocessed;
    localMapProcessed.add("footscore_polygon");

    //copy command velocity
    boost::unique_lock<boost::shared_mutex> lockCommandVelocity(mutexCommandVelocity_);
    LinearVelocity commandVelocityVector(commandVelocity_);
    lockCommandVelocity.unlock();

    //calculate Line footscores
    if (commandVelocityVector.vector().norm() != 0) {
      commandVelocityVector.vector().normalize();
    }

    // First iteraton through whole map.
    for (grid_map::GridMapIterator iterator(localMapUnprocessed); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      grid_map::Position candidateFoothold;
      localMapUnprocessed.getPosition(index, candidateFoothold);
      bool isGood = true;

      grid_map::Position secondLinePoint(candidateFoothold.x() + circleDistancePolygon_ * commandVelocityVector.x(),
                                         candidateFoothold.y() + circleDistancePolygon_ * commandVelocityVector.y());

      grid_map::Position tempPosition;

      //second iteration over map values in in line
      for (grid_map::LineIterator iterator(localMapUnprocessed, candidateFoothold, secondLinePoint);
          !iterator.isPastEnd(); ++iterator) {

        const grid_map::Index indexLine(*iterator);
        bool tempPositionWithinMap = localMapUnprocessed.getPosition(indexLine, tempPosition);

        if (tempPositionWithinMap) {
          if (localMapUnprocessed.isValid(indexLine, "footscore_binary")
              && localMapUnprocessed.at("footscore_binary", indexLine) == 0.0) {
            isGood = false;
          }
        }
      }

      if (isGood) {
        localMapProcessed.at("footscore_polygon", index) = 1.0;
      } else {
        localMapProcessed.at("footscore_polygon", index) = 0.0;
      }
    }

    {
      //copy local processed map into mapProcessed_
      boost::unique_lock<boost::shared_mutex> lockProcessedMap(mutexProcessedMap_);
      mapProcessed_ = localMapProcessed;
      mapProcessedAvailable_ = true;
    }

//    //Timing
//    timer.splitTime("lineMapProcessing");
//    std::cout << timer << std::endl;
//    std::cout << "averageTime: "<<timer.getAverageElapsedTimeUSec("lineMapProcessing")<< std::endl;
//
//    double timeSum = nMeasurements_*averageTime_;
//    timeSum +=timer.getAverageElapsedTimeUSec("lineMapProcessing");
//    nMeasurements_++;
//    averageTime_= timeSum/nMeasurements_;
//    std::cout<<"averageTime: "<<averageTime_<<std::endl;
  }

  return true;
}

bool TerrainModelElevationMap::getNormal(const loco::Position& positionWorldToLocationInWorldFrame,
                                         loco::Vector& normalInWorldFrame) const {
  return freePlaneModel_.getNormal(positionWorldToLocationInWorldFrame, normalInWorldFrame);
}

void TerrainModelElevationMap::getFootholdsForVisualization(const unsigned int legId,
                                                            Position& invertedPendulumFootholds,
                                                            Position& binaryFootholds, Position& polygonFootholds,
                                                            double& searchRadius, double& lineLength,
                                                            LinearVelocity& commandVelocity,
                                                            grid_map::GridMap& map) const {
  invertedPendulumFootholds = invertedPendulumFootholds_[legId];
  binaryFootholds = binaryFootholds_[legId];
  polygonFootholds = polygonFootholds_[legId];
  searchRadius = footholdSearchRadius_;
  lineLength = circleDistancePolygon_;
  commandVelocity = commandVelocity_;
  if (mapProcessedAvailable_) {
    map = mapProcessed_;
  }
}

loco::Position TerrainModelElevationMap::getNearestSuitableFoothold(const Position& positionWorldToFootholdInWorldFrame,
                                                                    const loco::LegBase* leg) {
  grid_map::Position originalFoothold(positionWorldToFootholdInWorldFrame.x(), positionWorldToFootholdInWorldFrame.y());
  grid_map::Position newFoothold = originalFoothold;
  grid_map::Position polygonFoothold;
  bool polygonFootholdFound = false;
  bool positionWithinMap = false;

  invertedPendulumFootholds_[leg->getId()] = positionWorldToFootholdInWorldFrame;

  if (mapUnprocessedAvailable_) {
    positionWithinMap = mapUnprocessed_.isInside(originalFoothold);
  }

  //if a map is available and the position of the original foothold is within map,
  //search for best foothold. Else keep original foothold
  if (positionWithinMap && mapUnprocessedAvailable_) {

    //check if original candidate is good

    grid_map::Index index;
    mapUnprocessed_.getIndex(originalFoothold, index);
    bool originalFootholdIsValid = false;

    double original_score = 0.0;
    if (mapUnprocessed_.isValid(index, "footscore_binary")) {
      original_score = mapUnprocessed_.atPosition("footscore_binary", originalFoothold);
      if (original_score == 1.0) {
        originalFootholdIsValid = true;
        binaryFootholds_[leg->getId()] = positionWorldToFootholdInWorldFrame;
      }
    }

    if (mapProcessedAvailable_ && original_score == 1.0) {
      boost::unique_lock<boost::shared_mutex> lockProcessedMap(mutexProcessedMap_);
      double polygon_score = mapProcessed_.atPosition("footscore_polygon", originalFoothold);
      if (polygon_score == 0.0) {
        originalFootholdIsValid = false;
      } else {
        polygonFootholds_[leg->getId()] = positionWorldToFootholdInWorldFrame;
      }
    }

    if (!originalFootholdIsValid) {

      grid_map::Position tempFoothold;
      double sqrDist = 0.0;
      double minSqrDist = footholdSearchRadius_ * footholdSearchRadius_;
      double minSqrDistPolygon = footholdSearchRadius_ * footholdSearchRadius_;

      for (grid_map::CircleIterator iterator(mapUnprocessed_, originalFoothold, footholdSearchRadius_);
          !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);

        bool tempFootholdWithinMap = mapUnprocessed_.getPosition(index, tempFoothold);

        if (tempFootholdWithinMap) {
          sqrDist = (originalFoothold.x() - tempFoothold.x()) * (originalFoothold.x() - tempFoothold.x())
              + (originalFoothold.y() - tempFoothold.y()) * (originalFoothold.y() - tempFoothold.y());

          if (mapUnprocessed_.isValid(index, "footscore_binary")
              && mapUnprocessed_.at("footscore_binary", index) == 1.0) {

            if (sqrDist < minSqrDist) {
              newFoothold = tempFoothold;
              minSqrDist = sqrDist;

            }
            //Use processed map for polygon footholds
            if (mapProcessedAvailable_) {
              boost::unique_lock<boost::shared_mutex> lockProcessedMap(mutexProcessedMap_);
              if (mapProcessed_.isInside(tempFoothold)) {
                if (mapProcessed_.atPosition("footscore_polygon", tempFoothold) == 1.0 && sqrDist < minSqrDistPolygon) {
                  polygonFoothold = tempFoothold;
                  polygonFootholdFound = true;
                  minSqrDistPolygon = sqrDist;
                }
              }
            }
          }
        }
      }
      loco::Position binaryFoothold(newFoothold.x(), newFoothold.y(), 0.0);
      getHeight(binaryFoothold);

      binaryFootholds_[leg->getId()] = binaryFoothold;
      if (polygonFootholdFound) {
        newFoothold = polygonFoothold;
        loco::Position polygonFoothold(newFoothold.x(), newFoothold.y(), 0.0);
        getHeight(polygonFoothold);
        polygonFootholds_[leg->getId()] = polygonFoothold;
      } else {
        MELO_WARN_STREAM("No PolygonFootholdFound!!");
        polygonFootholds_[leg->getId()] = loco::Position::Zero();
      }
    }
  }

  loco::Position positionWorldToNearestSuitableFootholdInWorldFrame(newFoothold.x(), newFoothold.y(), 0.0);
  getHeight(positionWorldToNearestSuitableFootholdInWorldFrame);

  return positionWorldToNearestSuitableFootholdInWorldFrame;
}  //Search for nearest suitable Foothold

bool TerrainModelElevationMap::getMaxHeightBetweenTwoPoints(const loco::Position& point1, const loco::Position& point2,
                                                            double& height) const {

  //If a map is available: Use line iterator to find largest value on the line
  if (mapUnprocessedAvailable_) {
    grid_map::Position startPosition(point1.x(), point1.y());
    grid_map::Position endPosition(point2.x(), point2.y());
    grid_map::Position tempPosition;
    double maxHeight = 0.0;

    for (grid_map::LineIterator iterator(mapUnprocessed_, startPosition, endPosition); !iterator.isPastEnd();
        ++iterator) {
      const grid_map::Index index(*iterator);

      bool tempPositionWithinMap = mapUnprocessed_.getPosition(index, tempPosition);

      if (tempPositionWithinMap) {
        if (mapUnprocessed_.isValid(index, heightLayerName_)
            && mapUnprocessed_.at(heightLayerName_, index) > maxHeight) {
          maxHeight = mapUnprocessed_.at(heightLayerName_, index);
        }
      }
    }
    height = maxHeight;
  } else {
    // If no map is available: the maximum is the higher of both points.
    return freePlaneModel_.getMaxHeightBetweenTwoPoints(point1, point2, height);
  }
  return true;
}

bool TerrainModelElevationMap::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const {

  grid_map::Position position(positionWorldToLocationInWorldFrame.x(), positionWorldToLocationInWorldFrame.y());
  grid_map::Index index;
  bool positionWithinMap = false;

  //check if Map value is available, use map when possible
  if (mapUnprocessedAvailable_) {
    positionWithinMap = mapUnprocessed_.isInside(position);
  }

  if (positionWithinMap && mapUnprocessedAvailable_) {
    mapUnprocessed_.getIndex(position, index);

    if (mapUnprocessed_.isValid(index, heightLayerName_)) {
      double heightFromMap = mapUnprocessed_.atPosition(heightLayerName_, position);

      if (heightFromMap > -0.3 && heightFromMap < 0.3) {
        positionWorldToLocationInWorldFrame.z() = heightFromMap;
      } else {
        getHeightFromPlane(positionWorldToLocationInWorldFrame);
      }
    } else {
      getHeightFromPlane(positionWorldToLocationInWorldFrame);
    }
  } else {
    getHeightFromPlane(positionWorldToLocationInWorldFrame);
  }

  return true;
}

bool TerrainModelElevationMap::getHeight(const loco::Position& positionWorldToLocationInWorldFrame,
                                         double& heightInWorldFrame) const {
  grid_map::Position position(positionWorldToLocationInWorldFrame.x(), positionWorldToLocationInWorldFrame.y());
  bool positionWithinMap = false;

  //check if Map value is available, use map when possible
  if (mapUnprocessedAvailable_) {
    positionWithinMap = mapUnprocessed_.isInside(position);
  }

  if (positionWithinMap && mapUnprocessedAvailable_) {
    grid_map::Index index;
    mapUnprocessed_.getIndex(position, index);
    if (mapUnprocessed_.isValid(index, heightLayerName_)) {
      double heightFromMap = mapUnprocessed_.atPosition(heightLayerName_, position);
      if (heightFromMap > -0.3 && heightFromMap < 0.3) {
        heightInWorldFrame = heightFromMap;
      } else {
        getHeightFromPlane(positionWorldToLocationInWorldFrame, heightInWorldFrame);
      }
    } else {
      getHeightFromPlane(positionWorldToLocationInWorldFrame, heightInWorldFrame);
    }
  } else {
    getHeightFromPlane(positionWorldToLocationInWorldFrame, heightInWorldFrame);
  }

  return true;
}

bool TerrainModelElevationMap::getHeightFromPlane(
    loco::Position& positionWorldToLocationInWorldFrame) const {
  return freePlaneModel_.getHeight(positionWorldToLocationInWorldFrame);
}

bool TerrainModelElevationMap::getHeightFromPlane(
    const loco::Position& positionWorldToLocationInWorldFrame,
    double& heightInWorldFrame) const {
  return freePlaneModel_.getHeight(positionWorldToLocationInWorldFrame, heightInWorldFrame);
}

bool TerrainModelElevationMap::getFrictionCoefficientForFoot(
    const loco::Position& positionWorldToLocationInWorldFrame,
    double& frictionCoefficient) const {
  return freePlaneModel_.getFrictionCoefficientForFoot(
      positionWorldToLocationInWorldFrame, frictionCoefficient);
}

Position TerrainModelElevationMap::getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(
    const Position& positionInWorldFrame) const {
  double planeHeightAtPosition = 0.0;
  freePlaneModel_.getHeight(positionInWorldFrame, planeHeightAtPosition);

  Vector n;
  freePlaneModel_.getNormal(positionInWorldFrame, n);

  const double distanceFromSurfaceAlongSurfaceNormal =
  freePlaneModel_.getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(positionInWorldFrame);

  if (positionInWorldFrame.z() > planeHeightAtPosition) {
    return (positionInWorldFrame - distanceFromSurfaceAlongSurfaceNormal * Position(n));
  } else if (positionInWorldFrame.z() < planeHeightAtPosition) {
    return (positionInWorldFrame + distanceFromSurfaceAlongSurfaceNormal * Position(n));
  } else {
    return positionInWorldFrame;
  }
}

double TerrainModelElevationMap::getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(
    const Position& positionInWorldFrame) const {
  return freePlaneModel_.getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(positionInWorldFrame);
}

double TerrainModelElevationMap::getHeightAboveTerrainAlongSurfaceNormal(
    const Position& positionWorldToLocationInWorldFrame) const {
  return freePlaneModel_.getHeightAboveTerrainAlongSurfaceNormal(positionWorldToLocationInWorldFrame);
}

bool TerrainModelElevationMap::addVariablesToLog(const std::string& ns) const {
  return freePlaneModel_.addVariablesToLog(ns);
}

} /* namespace loco */


