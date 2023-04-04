/*
 * interactive_test.cpp
 *
 *  Created on: Aug 13, 2017
 *      Author: Perry Franklin
 */

#include <kindr/common/gtest_eigen.hpp>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <gtest/gtest.h>
#include <kindr_ros/RosGeometryMsgPose.hpp>

#include <boost/bind.hpp>
#include <string>

#include "test_declarations.hpp"

#include <collisions_visualization/geometry_to_marker.hpp>
#include <collisions_fcl/ColliderManagerFcl.hpp>

using namespace kindr;
using namespace collisions_fcl;

namespace collisions_fcl_tests{

class BodyMarker : public visualization_msgs::InteractiveMarker
{
 public:
  BodyMarker(const std::shared_ptr<const TestRigidBody>& item)
      : scale_(1.0),
        frameId_("world"),
        item_(item)
  {
    color_.r = 1.0;
    color_.a = 0.6;
    setup(item);
  }

  ~BodyMarker() {}

  void updatePose(){
    HomTransformMatrixD initial_pose = item_->getTransform();
    kindr_ros::convertToRosGeometryMsg(initial_pose, pose);
  }

  void color(bool colliding){
    if (colliding){
      for (visualization_msgs::Marker& marker: controls[0].markers){
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      }
    }
    else
    {
      for (visualization_msgs::Marker& marker: controls[0].markers){
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      }
    }
  }

 private:

  void setup(const std::shared_ptr<const TestRigidBody>& item)
  {
    controls.clear();
    menu_entries.clear();

    header.frame_id = frameId_;
    name = item->getName();
    description = item->getName();
    scale = scale_;
    HomTransformMatrixD initial_pose = item->getTransform();
    kindr_ros::convertToRosGeometryMsg(initial_pose, pose);

    visualization_msgs::InteractiveMarkerControl mainControl;
    mainControl.name = name;
    mainControl.always_visible = true;

    if (item->collision_geometry){
      visualization_msgs::MarkerArray geomMarker = collisions_visualization::CollisionsGeomMarkerConverter<CoordinateFrame>::geomToMarker(*item->collision_geometry);
      for (visualization_msgs::Marker& marker: geomMarker.markers){
        marker.color = color_;
        marker.header.frame_id = "";
      }
      mainControl.markers = geomMarker.markers;
      controls.push_back(mainControl);
    }
    // Add interactive controls.
    visualization_msgs::InteractiveMarkerControl positionControl;
    positionControl.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

    // Move about x-axis.
    positionControl.orientation.w = 1;
    positionControl.orientation.x = 1;
    positionControl.orientation.y = 0;
    positionControl.orientation.z = 0;
    positionControl.name = "move_x";
    positionControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls.push_back(positionControl);

    // Move about y-axis.
    positionControl.orientation.w = 1;
    positionControl.orientation.x = 0;
    positionControl.orientation.y = 1;
    positionControl.orientation.z = 0;
    positionControl.name = "move_y";
    positionControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls.push_back(positionControl);

    // Move about z-axis.
    positionControl.orientation.w = 1;
    positionControl.orientation.x = 0;
    positionControl.orientation.y = 0;
    positionControl.orientation.z = 1;
    positionControl.name = "move_z";
    positionControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls.push_back(positionControl);

    // Rotate about x-axis.
    positionControl.orientation.w = 1;
    positionControl.orientation.x = 1;
    positionControl.orientation.y = 0;
    positionControl.orientation.z = 0;
    positionControl.name = "move_x";
    positionControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    controls.push_back(positionControl);

    // Rotate about y-axis.
    positionControl.orientation.w = 1;
    positionControl.orientation.x = 0;
    positionControl.orientation.y = 1;
    positionControl.orientation.z = 0;
    positionControl.name = "move_y";
    positionControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    controls.push_back(positionControl);

    // Rotate about z-axis.
    positionControl.orientation.w = 1;
    positionControl.orientation.x = 0;
    positionControl.orientation.y = 0;
    positionControl.orientation.z = 1;
    positionControl.name = "move_z";
    positionControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    controls.push_back(positionControl);
  }

  double scale_;
  std_msgs::ColorRGBA color_;
  std::string frameId_;
  std::shared_ptr<const TestRigidBody> item_;
};

class CollisionsInteractiveHelper
{
 public:
  CollisionsInteractiveHelper(std::shared_ptr<TestRigidBody> rigid_body_ptr, interactive_markers::InteractiveMarkerServer* marker_server ):
        markerServer_(marker_server),
        rigid_body_ptr_(rigid_body_ptr),
        name_(rigid_body_ptr->getName()),
        marker_(rigid_body_ptr)
  {
    markerServer_->insert(marker_, boost::bind(&CollisionsInteractiveHelper::positionFeedback, this, _1));
    markerServer_->applyChanges();
//     First run at initial position.
    visualization_msgs::InteractiveMarkerFeedbackConstPtr emptyFeedback(
        new visualization_msgs::InteractiveMarkerFeedback());
    positionFeedback(emptyFeedback);
  }

  void color(bool colliding){
    marker_.color(colliding);
    marker_.updatePose();
    markerServer_->insert(marker_);
    markerServer_->applyChanges();
  }

 protected:

  void positionFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if (feedback->marker_name == name_){
      HomTransformMatrixD pose;
      kindr_ros::convertFromRosGeometryMsg(feedback->pose, pose);
      rigid_body_ptr_->setTransform(pose);
    }
  }

 private:
  interactive_markers::InteractiveMarkerServer* markerServer_;

  BodyMarker marker_;

  std::string name_;
  std::shared_ptr<TestRigidBody> rigid_body_ptr_;
};

class CollisionsInteractiveTestRosHandler{

 public:

  CollisionsInteractiveTestRosHandler():
    marker_server_("collisions_fcl"){
    ros::NodeHandle nh;

    collider = new collisions_fcl::ColliderManagerFcl<CoordinateFrame>;

    fillTestBodyContainer(rigid_body_container_);

    collisions_geometry::CollisionGeometry::Transform identity;
    rigid_body_container_[BodyEnum::BODY0]->collision_geometry.reset(new collisions_geometry::CollisionGeometryBox(1,0.5,0.3,identity.getPosition(), identity.getRotation()));
    rigid_body_container_[BodyEnum::BODY1]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCapsule(0.5,1.0,identity.getPosition(), identity.getRotation()));
    rigid_body_container_[BodyEnum::BODY2]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCylinder(0.5,1.0,identity.getPosition(), identity.getRotation()));

    { // Combo Geom, scoped to prevent unexpected naming clashes (eg stupid mistakes)
      std::shared_ptr<collisions_geometry::CollisionGeometryCombo> combo_geom( new collisions_geometry::CollisionGeometryCombo());
      rigid_body_container_[BodyEnum::BODY3]->collision_geometry = combo_geom;

      std::unique_ptr<collisions_geometry::CollisionGeometry> box_geom(new collisions_geometry::CollisionGeometryBox(1,0.5,0.3,identity.getPosition(), identity.getRotation()));
      std::unique_ptr<collisions_geometry::CollisionGeometry> box2_geom(new collisions_geometry::CollisionGeometryBox(0.5,1.5,0.3,identity.getPosition(), identity.getRotation()));

      combo_geom->addGeometry(box_geom);
      combo_geom->addGeometry(box2_geom);
    }

    helpers.push_back( new CollisionsInteractiveHelper(std::static_pointer_cast<TestRigidBody>(rigid_body_container_[BodyEnum::BODY0]), &marker_server_) );
    helpers.push_back( new CollisionsInteractiveHelper(std::static_pointer_cast<TestRigidBody>(rigid_body_container_[BodyEnum::BODY1]), &marker_server_) );
    helpers.push_back( new CollisionsInteractiveHelper(std::static_pointer_cast<TestRigidBody>(rigid_body_container_[BodyEnum::BODY2]), &marker_server_) );
    helpers.push_back( new CollisionsInteractiveHelper(std::static_pointer_cast<TestRigidBody>(rigid_body_container_[BodyEnum::BODY3]), &marker_server_) );

    timer = nh.createTimer(ros::Duration(0.1), &CollisionsInteractiveTestRosHandler::timerCallback, this);
  }
  ~CollisionsInteractiveTestRosHandler(){
    if (collider){
      delete collider;
    }
    for (CollisionsInteractiveHelper* helper :  helpers){
      delete helper;
    }
  }

protected:

 void timerCallback(const ros::TimerEvent&){

   TestCollisionGroup group1({BodyEnum::BODY0, BodyEnum::BODY1}, rigid_body_container_);
   TestCollisionGroup group2({BodyEnum::BODY2, BodyEnum::BODY3}, rigid_body_container_);

   auto results = collider->checkCollision(group1, group2);

   std::set<BodyEnum> not_list = {BodyEnum::BODY0, BodyEnum::BODY1, BodyEnum::BODY2, BodyEnum::BODY3};
   std::set<BodyEnum> col_list;
   for (auto colliding_pair: results.colliding_pairs){

     not_list.erase(colliding_pair.first);
     not_list.erase(colliding_pair.second);

     col_list.insert(colliding_pair.first);
     col_list.insert(colliding_pair.second);

   }

   for (BodyEnum not_col: not_list){
     helpers[(int) not_col]->color(false);
   }

   for (BodyEnum is_col: col_list){
     helpers[(int) is_col]->color(true);
   }

 }

private:

 std::vector<CollisionsInteractiveHelper*> helpers;
 TestRigidBodyContainer rigid_body_container_;

 interactive_markers::InteractiveMarkerServer marker_server_;

 collisions::ColliderManager<CoordinateFrame>* collider;

 ros::Timer timer;

};

class CollisionsInteractiveTest : public ::testing::Test {

protected:

  static void SetUpTestCase()
  {

    ros_handler_ = new CollisionsInteractiveTestRosHandler;

  }

  static void TearDownTestCase()
  {
    delete ros_handler_;
    ros_handler_ = nullptr;
  }

  static CollisionsInteractiveTestRosHandler* ros_handler_;
};

CollisionsInteractiveTestRosHandler* CollisionsInteractiveTest::ros_handler_ = nullptr;

TEST_F(CollisionsInteractiveTest, Main)
{
  ros::spin();
}

}

