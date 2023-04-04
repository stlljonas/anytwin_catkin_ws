/*
 * polygon_visual.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: Perry Franklin
 */

#include "anymal_rviz_plugin/polygon_visual.hpp"

namespace anymal_rviz_plugin {

PolygonVisual::PolygonVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager), frame_node_(parent_node->createChildSceneNode()), enabled_(true), thickness_(0.01), filled_(false) {
  edges_ = std::make_unique<rviz::BillboardLine>(scene_manager_, frame_node_);
  fill_mesh_ = std::make_unique<rviz::MeshShape>(scene_manager_, frame_node_);
}

PolygonVisual::~PolygonVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}

void PolygonVisual::setVertices(const std::vector<Ogre::Vector3>& points) {
  points_ = points;
}

void PolygonVisual::drawVisual() {
  if (enabled_) {
    edges_->clear();

    if (!points_.empty()) {
      edges_->setLineWidth(thickness_);
      edges_->setColor(color_.r, color_.g, color_.b, color_.a);
      for (std::vector<Ogre::Vector3>::const_iterator it = points_.begin(); it != points_.end(); it++) {
        edges_->addPoint(*it);
      }
    }

    fill_mesh_->clear();
    // The fill mesh just uses a simple fan algorithm for now, which will fail horribly for non-convex
    if (filled_) {
      if (points_.size() >= 3) {
        fill_mesh_->estimateVertexCount(points_.size());
        fill_mesh_->setColor(color_.r, color_.g, color_.b, color_.a);
        for (size_t i = 0; i < points_.size(); ++i) {
          fill_mesh_->addVertex(points_[i]);
        }
        fill_mesh_->beginTriangles();

        // The fan algorithm for triangulating a polygon
        for (size_t i = 1; i < points_.size() - 1; ++i) {
          fill_mesh_->addTriangle(0, i, i + 1);
        }
        fill_mesh_->endTriangles();
      }
    }
  }
}

void PolygonVisual::setEnabled(bool enabled) {
  enabled_ = enabled;
  if (!enabled_) {
    edges_->clear();
    fill_mesh_->clear();
  } else {
    drawVisual();
  }
}

void PolygonVisual::setFilled(bool fill) {
  filled_ = fill;
}

void PolygonVisual::setLineColorAlpha(const Ogre::ColourValue& color) {
  color_ = color;
  drawVisual();
}

void PolygonVisual::setLineThickness(float thickness) {
  thickness_ = thickness;
  drawVisual();
}

}  // namespace anymal_rviz_plugin
