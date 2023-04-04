/*
 * SupportPolygon.cpp
 *
 *  Created on: Sep 3, 2015
 *      Author: Christian Gehring
 */

#include "loco_ros/visualization/SupportPolygon.hpp"

namespace loco_ros {

SupportPolygon::SupportPolygon()
{

}

SupportPolygon::~SupportPolygon()
{

}

bool SupportPolygon::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {

  publisher_ = nodeHandle.advertise<visualization_msgs::Marker>(topic, 100);

  supportPolygon_.pose.orientation.w = 1.0;
  supportPolygon_.action = visualization_msgs::Marker::ADD;
  supportPolygon_.type = visualization_msgs::Marker::LINE_LIST;
  supportPolygon_.scale.x = 0.005;
  supportPolygon_.scale.y = 0.005;
  supportPolygon_.color.a = 1.0;
  supportPolygon_.color.r = 0.0;
  supportPolygon_.color.g = 0.0;
  supportPolygon_.color.b = 1.0;
  supportPolygon_.id = 1;
  supportPolygon_.header.frame_id = "odom";
  supportPolygon_.ns = "support_polygon";

  return true;
}

bool SupportPolygon::shutdown() {
  publisher_.shutdown();
  return true;
}

bool SupportPolygon::update(const loco::TorsoBase& torso, const loco::Legs& legs, bool projectOnXYPlane) {

  if (publisher_.getNumSubscribers() > 0u) {
    supportPolygon_.header.stamp = ros::Time();
    supportPolygon_.points.clear();

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;


    for (auto leg : legs) {
     if (leg->getContactSchedule().isGrounded()) {
       const loco::Position& position = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
       p1.x = position.x();
       p1.y = position.y();
       if (projectOnXYPlane) {
         p1.z = 0.0;
       }
       else {
         p1.z = position.z();
       }


       for (auto otherLeg : legs) {
         if (otherLeg == leg) {
           continue;
         }
         if (otherLeg->getContactSchedule().isGrounded()) {
           const loco::Position& position2 = otherLeg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
           p2.x = position2.x();
           p2.y = position2.y();
           if (projectOnXYPlane) {
             p2.z = 0.0;
           }
           else {
             p2.z = position2.z();
           }
           supportPolygon_.points.push_back(p1);
           supportPolygon_.points.push_back(p2);
         }
       }
     }
    }


    visualization_msgs::MarkerConstPtr msg(new visualization_msgs::Marker(supportPolygon_));
    publisher_.publish(msg);
  }
  return true;
}

bool SupportPolygon::publish() {
  if (publisher_.getNumSubscribers() > 0u) {
    visualization_msgs::MarkerConstPtr msg(new visualization_msgs::Marker(supportPolygon_));
    publisher_.publish(msg);
  }
  return true;
}

unsigned int SupportPolygon::getNumSubscribers() const {
  return publisher_.getNumSubscribers();
}

} /* namespace loco_ros */
