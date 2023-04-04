/*
 * SteeringControllerRos.cpp
 *
 *  Created on: Okt 8, 2017
 *  Author: Gabriel Hottiger
 */

// loco_ros
#include "loco_ros/SteeringControllerRos.hpp"

// any_node
#include "any_node/Topic.hpp"

// kindr_ros
#include "kindr_ros/RosGeometryMsgPhysicalQuantities.hpp"

namespace loco_ros {

SteeringControllerRos::SteeringControllerRos()
    : dt_(0.0),
      steeringControllerMutex_(nullptr),
      steeringController_(nullptr),
      getActiveSteeringControllerServer_(),
      getAvailableSteeringControllersServer_(),
      switchSteeringControllerServer_(),
      notifySteeringModePublisher_(),
      activeSteeringModeMsg_(),
      steeringVisualizationPublisher_(),
      positionWorldToCoRInWorldFrameMsg_(),
      steeringLinesInWorldFrameMsg_()
{

}

bool SteeringControllerRos::initialize(ros::NodeHandle& nodeHandle,
                                       std::mutex* steeringControllerMutex,
                                       loco::SteeringControllerModule* steeringController,
                                       double dt) {

  // Copy pointers / timestep
  steeringControllerMutex_ = steeringControllerMutex;
  steeringController_ = steeringController;
  dt_ = dt;

  getActiveSteeringControllerServer_ = any_node::advertiseService(nodeHandle,
                                                                  "get_active_steering_controller",
                                                                  "get_active_steering_controller",
                                                                  &SteeringControllerRos::getActiveSteeringController,
                                                                  this);
  getAvailableSteeringControllersServer_ = any_node::advertiseService(nodeHandle,
                                                                      "get_available_steering_controllers",
                                                                      "get_available_steering_controllers",
                                                                      &SteeringControllerRos::getAvailableSteeringControllers,
                                                                      this);
  switchSteeringControllerServer_ = any_node::advertiseService(nodeHandle,
                                                               "switch_steering_controller",
                                                               "switch_steering_controller",
                                                               &SteeringControllerRos::switchSteeringController,
                                                               this);
  notifySteeringModePublisher_ = any_node::advertise<std_msgs::String>(nodeHandle,
                                                                       "active_steering_controller",
                                                                       "active_steering_controller",
                                                                       1);

  steeringVisualizationPublisher_ = any_node::advertise<visualization_msgs::Marker>(nodeHandle,
                                                                                    "steering_controller_visualization",
                                                                                    "steering_controller_visualization",
                                                                                    1);

  positionWorldToCoRInWorldFrameMsg_.header.frame_id = "odom";
  positionWorldToCoRInWorldFrameMsg_.type = visualization_msgs::Marker::SPHERE;
  positionWorldToCoRInWorldFrameMsg_.action = visualization_msgs::Marker::ADD;
  positionWorldToCoRInWorldFrameMsg_.ns = "CoR";
  positionWorldToCoRInWorldFrameMsg_.text = "CoR";
  positionWorldToCoRInWorldFrameMsg_.id = 0;
  positionWorldToCoRInWorldFrameMsg_.scale.x = 0.15;
  positionWorldToCoRInWorldFrameMsg_.scale.y = 0.15;
  positionWorldToCoRInWorldFrameMsg_.scale.z = 0.15;
  positionWorldToCoRInWorldFrameMsg_.color.a = 1.0;
  positionWorldToCoRInWorldFrameMsg_.color.g = 1.0;
  positionWorldToCoRInWorldFrameMsg_.pose.orientation.w = 1.0;


  steeringLinesInWorldFrameMsg_.header.frame_id = "odom";
  steeringLinesInWorldFrameMsg_.type = visualization_msgs::Marker::LINE_LIST;
  steeringLinesInWorldFrameMsg_.action = visualization_msgs::Marker::ADD;
  steeringLinesInWorldFrameMsg_.ns = "SteeringLines";
  steeringLinesInWorldFrameMsg_.text = "SteeringLines";
  steeringLinesInWorldFrameMsg_.id = 1;
  steeringLinesInWorldFrameMsg_.scale.x = 0.025;
  steeringLinesInWorldFrameMsg_.color.a = 1.0;
  steeringLinesInWorldFrameMsg_.color.g = 1.0;
  steeringLinesInWorldFrameMsg_.pose.orientation.w = 1.0;

  steeringController_->addObserver(this);

  return true;
}

bool SteeringControllerRos::updateMsgs(const loco::Legs & legs, const loco::TorsoBase & torso) {
  std::unique_lock<std::mutex> lk(*steeringControllerMutex_);
  // Calculate Center of rotation in world frame
  loco::Position positionControlToBaseProjectionInControlFrame(torso.getMeasuredState().inControlFrame().getPositionControlToBaseInControlFrame());
  positionControlToBaseProjectionInControlFrame.z() = 0.0;
  loco::Position positionWorldToCoRInWorldFrame = torso.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame() +
    torso.getMeasuredState().inControlFrame().getOrientationWorldToControl().inverseRotate(positionControlToBaseProjectionInControlFrame +
        steeringController_->getPositionBaseProjectionToCoRInControlFrame());

  // Convert CoR to ros message
  kindr_ros::convertToRosGeometryMsg(positionWorldToCoRInWorldFrame, positionWorldToCoRInWorldFrameMsg_.pose.position);

  // Resize steering lines to 2xnumLegs
  steeringLinesInWorldFrameMsg_.points.resize(2*legs.size());

  for(unsigned int i = 0; i<legs.size(); ++i) {
    // Get position in World Frame
    const loco::Position positionWorldToWheelInWorldFrame = legs.get(i).getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();

    // Get vector from wheel center to CoR
    const loco::Vector differenceWheelToCoRInWorldFrame(positionWorldToCoRInWorldFrame - positionWorldToWheelInWorldFrame);
    const loco::Vector orthogonalWheelDirectionInWorldFrame(
        legs.get(i).getEndEffector().getStateMeasured().getOrientationWorldToEndEffector().inverseRotate(loco::Vector::UnitY()));
    int sign = differenceWheelToCoRInWorldFrame.dot(orthogonalWheelDirectionInWorldFrame) > 0.0 ? 1 : -1;
    const loco::Position lineEndPoint = positionWorldToWheelInWorldFrame +
        loco::Position(sign * orthogonalWheelDirectionInWorldFrame * (1.0 + differenceWheelToCoRInWorldFrame.norm()));

    // Convert to ros msg
    kindr_ros::convertToRosGeometryMsg(positionWorldToWheelInWorldFrame, steeringLinesInWorldFrameMsg_.points.at(i*2));
    kindr_ros::convertToRosGeometryMsg(lineEndPoint, steeringLinesInWorldFrameMsg_.points.at(i*2 + 1));
  }

  return true;
}

void SteeringControllerRos::visualize() {
  loco_ros::publishMsg(steeringVisualizationPublisher_, positionWorldToCoRInWorldFrameMsg_);
  loco_ros::publishMsg(steeringVisualizationPublisher_, steeringLinesInWorldFrameMsg_);
}

void SteeringControllerRos::shutdown() {
  steeringController_->removeObserver(this);
  getActiveSteeringControllerServer_.shutdown();
  getAvailableSteeringControllersServer_.shutdown();
  switchSteeringControllerServer_.shutdown();
  notifySteeringModePublisher_.shutdown();
  steeringVisualizationPublisher_.shutdown();
}

void SteeringControllerRos::steeringModeChanged(const std::string& steeringMode) {
  std_msgs::String activeSteeringModeMsg;
  activeSteeringModeMsg.data = steeringMode;
  notifySteeringModePublisher_.publish(activeSteeringModeMsg);
}

bool SteeringControllerRos::switchSteeringController(rocoma_msgs::SwitchController::Request& req,
                                                     rocoma_msgs::SwitchController::Response& res) {
  std::lock_guard<std::mutex> lock(*steeringControllerMutex_);

  if( steeringController_->setActiveMode(req.name, dt_) ) {
    res.status = rocoma_msgs::SwitchController::Response::STATUS_SWITCHED;
  }
  else {
    res.status = rocoma_msgs::SwitchController::Response::STATUS_ERROR;
  }

  return true;
}

bool SteeringControllerRos::getActiveSteeringController(rocoma_msgs::GetActiveController::Request& req,
                                                        rocoma_msgs::GetActiveController::Response& res) {
  std::lock_guard<std::mutex> lock(*steeringControllerMutex_);
  res.active_controller = steeringController_->getActiveModeName();
  return true;
}

bool SteeringControllerRos::getAvailableSteeringControllers(rocoma_msgs::GetAvailableControllers::Request& req,
                                                            rocoma_msgs::GetAvailableControllers::Response& res) {
  std::lock_guard<std::mutex> lock(*steeringControllerMutex_);
  res.available_controllers = steeringController_->getModeNames();
  return true;
}

} /* namespace loco_ros */
