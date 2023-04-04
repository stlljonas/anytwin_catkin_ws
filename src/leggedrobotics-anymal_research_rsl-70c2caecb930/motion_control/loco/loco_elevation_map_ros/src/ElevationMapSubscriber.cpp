/*
 * ElevationMapSubscriber.cpp
 *
 *  Created on: Apr 18, 2017
 *      Author: Tanja Baumann
 */

// loco elevation map ros
#include "loco_elevation_map_ros/ElevationMapSubscriber.hpp"

namespace loco_ros {

ElevationMapSubscriber::ElevationMapSubscriber(ros::NodeHandle& nodeHandle,
                                               loco::TerrainModelElevationMap* terrainModel)
    : nodeHandle_(nodeHandle),
      mapTopic_("/footscore_estimation/footscore_map"),
      terrainModel_(terrainModel),
      gotNewMap_(false)

{

}

ElevationMapSubscriber::~ElevationMapSubscriber() {

}

bool ElevationMapSubscriber::initialize(double dt) {
  mapSubscriber_ = nodeHandle_.subscribe(mapTopic_, 1, &ElevationMapSubscriber::mapCallback, this);

  return true;
}

bool ElevationMapSubscriber::advance(double dt) {

  if (gotNewMap_) {
    boost::unique_lock<boost::shared_mutex> lock(mutexLocalMap_);
    terrainModel_->setMap(localMap_);
    gotNewMap_ = false;
  }

  return true;
}

void ElevationMapSubscriber::mapCallback(const grid_map_msgs::GridMap& map_msg) {
  //Convert gridmap_msg into gridmap.mutex localMap_ and copy new gridmap into it
  boost::unique_lock<boost::shared_mutex> lock(mutexLocalMap_);
  grid_map::GridMapRosConverter::fromMessage(map_msg, localMap_);

  gotNewMap_ = true;
}

}
