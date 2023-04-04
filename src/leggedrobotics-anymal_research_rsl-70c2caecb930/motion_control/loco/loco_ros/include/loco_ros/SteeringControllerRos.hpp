/*
 * TransformsM545.hpp
 *
 *  Created on: Feb 8, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/driving_control/SteeringController.hpp"
#include "loco/driving_control/SteeringControllerObserver.hpp"

//loco_ros
#include "loco_ros/loco_ros.hpp"

// rocoma msgs
#include "rocoma_msgs/GetActiveController.h"
#include "rocoma_msgs/GetAvailableControllers.h"
#include "rocoma_msgs/SwitchController.h"
#include "std_msgs/String.h"

// visualization_msgs
#include "visualization_msgs/Marker.h"

// ros
#include <ros/ros.h>

namespace loco_ros {

class SteeringControllerRos : public loco::SteeringControllerObserver {
 public:
  SteeringControllerRos();

  ~SteeringControllerRos() override = default;

  bool initialize(ros::NodeHandle& nodeHandle,
                  std::mutex* steeringControllerMutex,
                  loco::SteeringControllerModule* steeringController,
                  double dt);

  //! Update ros msgs
  bool updateMsgs(const loco::Legs & legs, const loco::TorsoBase & torso);

  //! Visualize steering lines
  void visualize();

  //! Shutdown ros communication
  void shutdown();

  /*! Notify function of the steering controller observer.
   * @param steeringMode new steering mode
   */
  void steeringModeChanged(const std::string& steeringMode) override;

  /*! Ros service to switch steering controller.
   * @param req desired steering controller
   * @param res status of the switch
   * @return true, if service executed
   */
  bool switchSteeringController(rocoma_msgs::SwitchController::Request& req,
                                rocoma_msgs::SwitchController::Response& res);

  /*! Ros service to get active steering controller.
   * @param req empty request
   * @param res active steering controller name
   * @return true, if service executed
   */
  bool getActiveSteeringController(rocoma_msgs::GetActiveController::Request& req,
                                   rocoma_msgs::GetActiveController::Response& res);

  /*! Ros service to get available steering modes.
   * @param req empty request
   * @param res vector of available steering controller names
   * @return true, if service executed
   */
  bool getAvailableSteeringControllers(rocoma_msgs::GetAvailableControllers::Request& req,
                                       rocoma_msgs::GetAvailableControllers::Response& res);


 protected:
  double dt_;
  std::mutex* steeringControllerMutex_;
  loco::SteeringControllerModule* steeringController_;

  ros::ServiceServer getActiveSteeringControllerServer_;
  ros::ServiceServer getAvailableSteeringControllersServer_;
  ros::ServiceServer switchSteeringControllerServer_;

  ros::Publisher notifySteeringModePublisher_;
  std_msgs::String activeSteeringModeMsg_;

  ros::Publisher steeringVisualizationPublisher_;
  visualization_msgs::Marker positionWorldToCoRInWorldFrameMsg_;
  visualization_msgs::Marker steeringLinesInWorldFrameMsg_;

};


} /* namespace loco_ros */
