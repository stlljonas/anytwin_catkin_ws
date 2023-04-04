/*
 * TerrainModelElevationMap.hpp
 *
 *  Created on: Feb 28, 2017
 *      Author: Tanja Baumann, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/TerrainModelPlane.hpp"
#include "loco/common/TerrainModelFreePlane.hpp"
#include <loco/common/legs/LegBase.hpp>

// grid map
#include "grid_map_core/grid_map_core.hpp"

// worker
#include "any_worker/Worker.hpp"

// boost
#include <boost/thread.hpp>

// stl
#include <memory>

class TiXmlHandle;

namespace loco {

class TerrainModelElevationMap : public TerrainModelPlane {

 public:
  TerrainModelElevationMap();
  virtual ~TerrainModelElevationMap() = default;

  /*! Initializes the plane at zero height and normal parallel to z-axis in world frame
   * @param dt  time step
   * @returns true
   */
  bool initialize(double dt) override;

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*! Gets the surface normal of the terrain at a certain position.
   * @param[in] positionWorldToLocationInWorldFrame the place to get the surface normal from (in the world frame)
   * @param[out] normalInWorldFrame the surface normal (in the world frame)
   * @return true if successful, false otherwise
   */
  virtual bool getNormal(const loco::Position& positionWorldToLocationInWorldFrame,
                         loco::Vector& normalInWorldFrame) const override;

  /*! Gets the maximal height of the terrain on a line between two points
   * (in world frame) from the map and sets the height
   * @param[in/out] point1, point2 as the points for the line, height to output the maximal height of the terrain
   * @return true if successful, false otherwise
   */
  bool getMaxHeightBetweenTwoPoints(const loco::Position& point1, const loco::Position& point2, double& height) const override;

  /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
   * (in world frame) from the map and sets the position.z() as the height (in world frame).
   * @param[in/out] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location in the map expressed in world frame
   * @return true if successful, false otherwise
   */
  bool getHeight(loco::Position& positionWorldToLocationInWorldFrame) const override;

  /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
   * (in world frame) from the map and sets heightInWorldFrame as the height (in world frame).
   * @param[in] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
   * @param[out] heightInWorldFrame   height in world frame evaluated from map at position positionWorldToLocationInWorldFrame
   * @return true if successful, false otherwise
   */
  bool getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const override;

  /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
   * (in world frame) from the plane and sets the position.z() as the height (in world frame).
   * @param[in/out] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location in the map expressed in world frame
   * @return true if successful, false otherwise
   */
  virtual bool getHeightFromPlane(loco::Position& positionWorldToLocationInWorldFrame) const;

  /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
   * (in world frame) from plane and sets heightInWorldFrame as the height (in world frame).
   * @param[in] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
   * @param[out] heightInWorldFrame   height in world frame evaluated from map at position positionWorldToLocationInWorldFrame
   * @return true if successful, false otherwise
   */
  virtual bool getHeightFromPlane(const loco::Position& positionWorldToLocationInWorldFrame,
                                  double& heightInWorldFrame) const;

  /*! Search for nearest suitable Foothold in an Area around the proposed Foothold
   *Returns the position with maximum suitability in the given deviation radius
   */
  virtual loco::Position getNearestSuitableFoothold(const loco::Position& positionWorldToFootholdInWorldFrame,
                                                    const loco::LegBase* leg);

  /*! Return friction coefficient for a foot at a certain position.
   * @param[in] positionWorldToLocationInWorldFrame position from origin of world frame to the requested location expressed in world frame
   * @param[out] frictionCoefficient friction coefficient evaluated at the given position
   * @return true if successful, false otherwise
   */
  virtual bool getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame,
                                             double& frictionCoefficient) const;

  /*! Set free plane parameters.
   * @param normal Normal vector to plane in world frame
   * @param position Point on plane in world frame
   */
  void setNormalandPositionInWorldFrame(const loco::Vector& normal, const loco::Position& position) override;

  /*! Set command velocity
   * @param commandVelocity Linear velocity from torso.getDesiredState().getLinearVelocityTarget
   */
  void setCommandVelocity(const LinearVelocity& commandVelocity);

  /*! Gets the height above the terrain (can also be negative), signed version of
   *  getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame()
   * @param positionWorldToLocationInWorldFrame   position for which height shall be calculated
   * @returns the height of the point above the terrain plane along the surface normal
   */
  virtual double getHeightAboveTerrainAlongSurfaceNormal(const Position& positionWorldToLocationInWorldFrame) const;

  /*! Set map.
   * @param map GridMap with height values
   */
  void setMap(const grid_map::GridMap& map);

  Position getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(const Position& position) const;

  void getFootholdsForVisualization(const unsigned int legId, Position& invertedPendulumFootholds,
                                    Position& binaryFootholds, Position& polygonFootholds, double& searchRadius,
                                    double& lineLength, LinearVelocity& commandVelocity, grid_map::GridMap& map) const;

  double getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(const Position& positionInWorldFrame) const;

  bool addVariablesToLog(const std::string& ns) const override;

 protected:

  virtual bool polygonMapProcessing();
  virtual bool lineMapProcessing();

  TerrainModelFreePlane freePlaneModel_;

  LinearVelocity commandVelocity_;

  double footholdSearchRadius_;
  double circleDistancePolygon_;
  double circleRadiusPolygon_;
  double workerTimestep_;
  int workerPriority_;

  grid_map::GridMap mapUnprocessed_;
  grid_map::GridMap mapProcessed_;

  bool mapUnprocessedAvailable_;
  bool mapProcessedAvailable_;

  std::string heightLayerName_;

  // Worker.
  std::unique_ptr<any_worker::Worker> worker_;
  boost::shared_mutex mutexUnprocessedMap_;
  boost::shared_mutex mutexProcessedMap_;
  boost::shared_mutex mutexCommandVelocity_;

  // Visualization.
  std::vector<loco::Position> invertedPendulumFootholds_;
  std::vector<loco::Position> binaryFootholds_;
  std::vector<loco::Position> polygonFootholds_;
};
// class

} /* namespace loco */
