/*
 * anymal_link_updater.hpp
 *
 *  Created on: Jan 12, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <memory>

#include <rviz/robot/link_updater.h>

#include <anymal_msgs/AnymalState.h>

#include <anymal_model/AnymalModel.hpp>

#include <urdf/model.h>

namespace anymal_rviz_plugin {

class AnymalLinkUpdater : public rviz::LinkUpdater {
 public:
  AnymalLinkUpdater(const std::string& urdf_description, const urdf::Model& urdf_model);
  virtual ~AnymalLinkUpdater();

  void update(const anymal_msgs::AnymalState& state);

  bool getLinkTransforms(const std::string& link_name, Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
                         Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation) const override;

  const anymal_model::AnymalModel& getModel() const;

 private:
  anymal_model::AnymalModel model_;

  urdf::Model urdf_model_;

  bool getLinkTransform(const std::string& link_name, kindr::HomTransformQuatD& transform) const;
};

}  // namespace anymal_rviz_plugin
