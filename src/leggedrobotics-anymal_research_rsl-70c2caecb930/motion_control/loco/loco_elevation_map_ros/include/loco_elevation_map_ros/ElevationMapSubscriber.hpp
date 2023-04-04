/*
 * ElevationMapSubscriber.hpp
 *
 *  Created on: Apr 18, 2017
 *      Author: Tanja Baumann
 */

#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include "grid_map_core/grid_map_core.hpp"

// ros
#include <ros/ros.h>

// loco elevation map
#include "loco_elevation_map/TerrainModelElevationMap.hpp"

// BOOST
#include <boost/thread.hpp>

namespace loco_ros {

class ElevationMapSubscriber {

 public:
  ElevationMapSubscriber(ros::NodeHandle& nodeHandle, loco::TerrainModelElevationMap* terrainModel);

  virtual ~ElevationMapSubscriber();

  virtual bool initialize(double dt);

  virtual bool advance(double dt);

 private:
  void mapCallback(const grid_map_msgs::GridMap& map_msg);

  grid_map::GridMap localMap_;
  boost::shared_mutex mutexLocalMap_;
  bool gotNewMap_;

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber mapSubscriber_;
  std::string mapTopic_;

  loco::TerrainModelElevationMap* terrainModel_;

};

}  // namespace
