#pragma once

// loco.
#include <loco/common/TerrainModelFreePlane.hpp>
#include <loco/common/TerrainModelPlane.hpp>
#include <loco/common/WholeBody.hpp>
#include <loco/common/typedefs.hpp>

// boost (for mutex).
#include <boost/thread.hpp>

// grid_map_core
#include <grid_map_core/grid_map_core.hpp>

// terrain sensing.
#include <terrain_sensing/terrain_sensing.hpp>

namespace loco {

class TerrainSensing : public TerrainModelPlane {
 public:
  explicit TerrainSensing(WholeBody& wholeBody);
  ~TerrainSensing() override = default;

  bool initialize(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;

  /*! Gets the maximal height of elevation map on a line between two points.
   * (in world frame) from the map and sets the height
   * @param[in/out] point1, point2 as the points for the line, height to output the maximal height of the terrain
   * @return true if successful, false otherwise
   */
  bool getMaxHeightBetweenTwoPoints(const Position& point1, const Position& point2, double& height) const override;

  /*! Gets the surface normal of the elevation map at a certain position using the elevation map.
   * @param[in] positionWorldToLocationInWorldFrame the place to get the surface normal from (in the world frame)
   * @param[out] normalInWorldFrame the surface normal (in the world frame)
   * @return true if successful, false otherwise
   */
  bool getNormal(const Position& positionWorldToLocationInWorldFrame, Vector& normalInWorldFrame) const override;

  /*! Gets the height of the elevation map at the a specific coordinate and sets the position.z() as the height.
   * @param[in/out] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location in the map expressed
   * in world frame
   * @return true if successful, false otherwise
   */
  bool getHeight(Position& positionWorldToLocationInWorldFrame) const override;

  /*! Gets the height of the elevation map at the a specific coordinate.
   * @param[in] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
   * @param[out] heightInWorldFrame   height in world frame evaluated from map at position positionWorldToLocationInWorldFrame
   * @return true if successful, false otherwise
   */
  bool getHeight(const Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const override;

  //! Set a new grid map.
  bool setMap(grid_map::GridMap& elevationMap);

  //! Return the grid map.
  const grid_map::GridMap& getMap() const;

  //! Returns the terrain model approximating the local terrain.
  const TerrainModelFreePlane& getFreePlaneModel() const;

  //! Set normal and position of the local terrain approximation.
  void setNormalandPositionInWorldFrame(const Vector& normal, const Position& position) override;

  //! Returns the friction coefficient of the terrain (local terrain approximation).
  bool getFrictionCoefficientForFoot(const Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const override;

  //! Projects a position onto the local terrain approximation along its plane normal.
  Position getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(const Position& positionWorldToFootholdInWorldFrame) const override;

  //! This function uses the local terrain approximation to compute the distance from the terrain.
  double getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(const Position& positionInWorldFrame) const override;

  //! This function uses the local terrain approximation to compute the height above the terrain.
  double getHeightAboveTerrainAlongSurfaceNormal(const Position& positionWorldToLocationInWorldFrame) const override;

  //! If false, height layer in grid map is used.
  void setUseHeightInformation(bool useHeightInformation) noexcept;

  //! If false, height layer in grid map is used.
  bool getUseHeightInformation() const noexcept;

  //! If true, foothold score in grid map is used.
  void setUseGradientInformation(bool useGradientInformation) noexcept;

  //! If true, foothold score in grid map is used.
  bool getUseGradientInformation() const noexcept;

  //! Get elevation map value at position for certain layer. Returns false if layer does not exist.
  bool getElevationMapValue(double& value, const terrain_sensing::GridLayers& layer,
                            const Position& positionWorldToLocationInWorldFrame) const noexcept;

  double getDistanceFeetOverElevationMap() const noexcept;

 protected:
  // Convert 3d to 2d position.
  grid_map::Position positionToGridMapPosition(const Position& pos) const;

  //! Whole body.
  const WholeBody& wholeBody_;

  //! Grid map.
  grid_map::GridMap gridMap_;

  //! Mutex for grid map.
  mutable boost::shared_mutex mutexGridMap_;

  //! Approximation of the terrain.
  TerrainModelFreePlane freePlaneModel_;

  //! Mutex for free plane model.
  mutable boost::shared_mutex mutexFreePlaneModel_;

  //! If false, height layer in grid map is used.
  bool useHeightInformation_;

  //! If true, foothold score in grid map is used.
  bool useGradientInformation_;

  //! Offset between foothold projected onto elevation map and measured point foot height.
  double distanceFeetOverElevationMap_;
};
}  // namespace loco
