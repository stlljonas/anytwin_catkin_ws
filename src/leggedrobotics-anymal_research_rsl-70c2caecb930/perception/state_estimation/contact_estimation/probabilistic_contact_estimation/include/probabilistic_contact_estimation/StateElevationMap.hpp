/*
 * StateElevationMap.hpp
 *
 *  Created on: Sept 01, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

// grid map.
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// probabilistic contact estimation
#include <probabilistic_contact_estimation/State.hpp>

// ros.
#include <ros/ros.h>

// C/C++ STD.
#include <mutex>
#include <atomic>

class TiXmlHandle;

namespace contact_estimation {

class StateElevationMap : public State {
  public:
    //! Constructor
    StateElevationMap(anymal_model::AnymalModel* model, ros::NodeHandle& nodeHandle);

    //! Default Destructor
    ~StateElevationMap() override;

    /** Loads parameters from an xml file
     *  @param handle tinyxml handle
     *  @return true, iff successful
     */
    bool loadParameters(const TiXmlHandle& handle) override;

    /** Initializes the module
     * @param dt  time step
     * @return true, iff successful
     */
    bool initialize(const double dt) override;

  protected:
    //! Subscribes to topics.
    void initializeSubscribers();

    //! Predicts touch-down location using elevation map.
    bool updateExpectedTouchDownPosition(const std_utils::EnumArray<AD::ContactEnum, ContactState>& contactState) override;

    //! Get height above terrain along gravity vector. The terrain height is extracted from the elevation map.
    double getHeight(const Eigen::Vector3d& positionWorldToLocationInWorldFrame) const override;

    //! Callback function used when subscribing to sub map topic.
    void copySubMap(const grid_map_msgs::GridMap::ConstPtr &subMap);

    //! If true, sub map and height layer are available.
    bool isHeightLayerAvailable() const;

    /**Get height from elevation map.
     *  @param location location at which the height should be extracted
     *  @param heightElevationMap height at location
     *  @return true, iff successful
     */
    bool getHeightFromElevationMapAtLocation(const grid_map::Position& location, double& heightElevationMap) const;

    //! Node handle.
    ros::NodeHandle nodeHandle_;

    //! Subscriber for elevation map.
    ros::Subscriber subMapCopySubscriber_;

    //! Local copy of sub map.
    grid_map::GridMap subMap_;

    //! Mutex for grid map.
    mutable std::mutex mutexSubMap_;

    //! Name of the height layer used to extract height information from the elevation map.
    std::string heightLayerName_;

    //! Name of subscriber topic for the elevation map.
    std::string elevationMapTopicName_;

    //! Distance between feed center and elevation map.
    double distanceFeetOverElevationMap_;

    //! If true, most recent update of the sub map was successful.
    std::atomic<bool> subMapAvailable_;
};

} /* namespace contact_estimation */
