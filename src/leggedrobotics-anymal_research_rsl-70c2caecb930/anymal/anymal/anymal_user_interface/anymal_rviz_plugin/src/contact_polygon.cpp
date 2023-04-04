/*
 * contact_polygon.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: Perry Franklin
 */

#include "anymal_rviz_plugin/contact_polygon.hpp"

namespace anymal_rviz_plugin {

ContactPolygon::ContactPolygon(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name,
                               rviz::Property* parent_property)
    : polygon_visual_(context->getSceneManager(), root_node),
      com_projection_visual_(context->getSceneManager(), root_node),
      com_polygon_visual_(context->getSceneManager(), root_node) {
  contact_polygon_property_ = new rviz::BoolProperty("Contact Polygon", true, "", parent_property, SLOT(enableContactPolygon()), this);

  line_color_property_ = new rviz::ColorProperty("Polygon Line Color", QColor(204, 204, 204), "Color of the Polygon Line visuals",
                                                 contact_polygon_property_, SLOT(changeLineColorAlpha()), this);

  line_alpha_property_ = new rviz::FloatProperty("Polygon Line alpha", 0.6, "Transparency of the Polygon Line visuals",
                                                 contact_polygon_property_, SLOT(changeLineColorAlpha()), this);
  line_alpha_property_->setMin(0.0);
  line_alpha_property_->setMax(1.0);

  line_thickness_property_ = new rviz::FloatProperty("Polygon Line Thickness", 0.01, "Thickness of the Polygon Line visuals",
                                                     contact_polygon_property_, SLOT(changeLineThickness()), this);
  line_thickness_property_->setMin(0.0);

  show_com_projection_property_ =
      new rviz::BoolProperty("Show CoM Projection", true, "Show a projection of the CoM onto the 'ground' contact polygon",
                             contact_polygon_property_, SLOT(enableComArrow()), this);

  com_projection_alpha_property_ = new rviz::FloatProperty("CoM Arrow Alpha", 0.6, "Transparency of the CoM Projection arrow",
                                                           contact_polygon_property_, SLOT(changeComArrowColorAlpha()), this);
  com_projection_alpha_property_->setMin(0.0);
  com_projection_alpha_property_->setMax(1.0);

  com_projection_color_property_ = new rviz::ColorProperty("CoM Arrow Color", QColor(0, 204, 204), "Color of the CoM Projection visuals",
                                                           contact_polygon_property_, SLOT(changeComArrowColorAlpha()), this);

  show_com_projection_column_property_ = new rviz::BoolProperty("Show CoM Column", true, "Enable the vertical line of the CoM Projection",
                                                                contact_polygon_property_, SLOT(enableComColumn()), this);

  com_projection_thickness_property_ = new rviz::FloatProperty("CoM Arrow Thickness", 0.03, "Thickness of the CoM Projection Arrow",
                                                               contact_polygon_property_, SLOT(changeComArrowThickness()), this);
  com_projection_thickness_property_->setMin(0.0);

  com_polygon_projection_alpha_property_ =
      new rviz::FloatProperty("CoM Polygon Projection Alpha", 0.6, "Transparency of the CoM Polygon Projection", contact_polygon_property_,
                              SLOT(changeComPolygonProjectionColorAlpha()), this);
  com_polygon_projection_alpha_property_->setMin(0.0);
  com_polygon_projection_alpha_property_->setMax(1.0);

  com_polygon_projection_color_property_ =
      new rviz::ColorProperty("CoM Polygon Projection Color", QColor(0, 102, 102), "Color of the CoM Polygon Projection ",
                              contact_polygon_property_, SLOT(changeComPolygonProjectionColorAlpha()), this);

  contact_polygon_property_->setDisableChildrenIfFalse(true);

  enableContactPolygon();
  changeLineColorAlpha();
  changeLineThickness();
  enableComArrow();
  changeComArrowColorAlpha();
  changeComArrowThickness();
  changeComPolygonProjectionColorAlpha();

  com_polygon_visual_.setFilled(true);
  com_polygon_visual_.setLineThickness(0.01);
}

ContactPolygon::~ContactPolygon() {
  delete line_color_property_;
  delete line_alpha_property_;
  delete line_thickness_property_;
  delete show_com_projection_property_;
  delete com_projection_color_property_;
  delete com_projection_alpha_property_;
  delete com_projection_thickness_property_;
  delete com_polygon_projection_color_property_;
  delete com_polygon_projection_alpha_property_;

  delete contact_polygon_property_;
}

void ContactPolygon::update(const AnymalLinkUpdater& updater) {
  const anymal_model::AnymalModel& model = updater.getModel();

  const romo::ContactShPtrContainer<anymal_description::ConcreteAnymalDescription>& contact_container = model.getContactContainer();

  std::vector<Ogre::Vector3> list_of_points;

  for (const std::shared_ptr<romo::Contact<anymal_description::ConcreteAnymalDescription>>& element : contact_container) {
    if (element->getState() != anymal_description::AnymalTopology::ContactStateEnum::OPEN) {
      romo::Position position = element->getPositionWorldToContact(anymal_model::AnymalModel::CoordinateFrameEnum::WORLD);
      list_of_points.emplace(list_of_points.end(), position.x(), position.y(), position.z());
    }
  }

  const std::vector<Ogre::Vector3> convexified_points = convexHull(list_of_points);

  polygon_visual_.setVertices(convexified_points);

  polygon_visual_.drawVisual();

  double floor_height_sum = 0.0;
  for (const Ogre::Vector3& point : list_of_points) {
    floor_height_sum += point.z;
  }
  double floor_height_average = floor_height_sum / list_of_points.size();
  com_projection_visual_.setFloor(floor_height_average);

  romo::Position com_position(model.getPositionWorldToCom(anymal_model::AnymalModel::CoordinateFrameEnum::WORLD));
  Ogre::Vector3 com_position_ogre(com_position.x(), com_position.y(), com_position.z());
  com_projection_visual_.setComPosition(com_position_ogre);

  std::vector<Ogre::Vector3> com_polygon_points = convexified_points;
  for (Ogre::Vector3& point : com_polygon_points) {
    point.z = floor_height_average;
  }
  com_polygon_visual_.setVertices(com_polygon_points);
  com_polygon_visual_.drawVisual();
}

void ContactPolygon::enableContactPolygon() {
  polygon_visual_.setEnabled(contact_polygon_property_->getBool());
  com_projection_visual_.setEnabled(contact_polygon_property_->getBool() && show_com_projection_property_->getBool());
  com_polygon_visual_.setEnabled(contact_polygon_property_->getBool() && show_com_projection_property_->getBool());
}

void ContactPolygon::changeLineColorAlpha() {
  Ogre::ColourValue color = rviz::qtToOgre(line_color_property_->getColor());
  color.a = line_alpha_property_->getFloat();
  polygon_visual_.setLineColorAlpha(color);
}

void ContactPolygon::changeLineThickness() {
  polygon_visual_.setLineThickness(line_thickness_property_->getFloat());
}

void ContactPolygon::enableComArrow() {
  com_projection_visual_.setEnabled(show_com_projection_property_->getBool());
  com_polygon_visual_.setEnabled(show_com_projection_property_->getBool());
}

void ContactPolygon::changeComArrowColorAlpha() {
  Ogre::ColourValue color(com_projection_color_property_->getColor().redF(), com_projection_color_property_->getColor().greenF(),
                          com_projection_color_property_->getColor().blueF(), com_projection_alpha_property_->getFloat());
  com_projection_visual_.setColor(color);
}

void ContactPolygon::enableComColumn() {
  com_projection_visual_.setEnableColumn(show_com_projection_column_property_->getBool());
}

void ContactPolygon::changeComArrowThickness() {
  com_projection_visual_.setWidth(com_projection_thickness_property_->getFloat());
}

void ContactPolygon::changeComPolygonProjectionColorAlpha() {
  Ogre::ColourValue color(com_polygon_projection_color_property_->getColor().redF(),
                          com_polygon_projection_color_property_->getColor().greenF(),
                          com_polygon_projection_color_property_->getColor().blueF(), com_polygon_projection_alpha_property_->getFloat());
  com_polygon_visual_.setLineColorAlpha(color);
}

std::vector<Ogre::Vector3> ContactPolygon::convexHull(const std::vector<Ogre::Vector3>& points) {
  return giftWrap(points);
}

// This uses the giftwrapping algorithm for finding a convex polygon in 2 dimensions
std::vector<Ogre::Vector3> ContactPolygon::giftWrap(const std::vector<Ogre::Vector3>& points) {
  std::vector<Ogre::Vector3> vertices;

  if (points.empty()) {
    return vertices;
  }

  // The first point should be the leftmost point
  vertices.push_back(points.front());
  for (const Ogre::Vector3& point : points) {
    if (point.x < vertices.front().x) {
      vertices.front() = point;
    }
  }

  // The giftwrapping algorithm tries to "wrap" around the points in the set.
  // If we have a point x_i on the convex hull, point x_i+1 is a point in the set s.t.
  // all other points in the set are on one side of it (in this implementation the right
  // side).
  // For a candidate p, we calculate c = R*(p-x_i), where R is a 90 degree rotation. We
  // then check all the points q in the set against it, checking for c*q < c*x_i. If
  // c*q > c*x_i, p is not the next convex hull point, and q becomes our candidate p.
  // Once we confirm that p is the next convex hull point,
  // For a more complete explanation: https://en.wikipedia.org/wiki/Gift_wrapping_algorithm
  do {
    const Ogre::Vector3& current_point = vertices.back();

    Ogre::Vector3 candidate_point;

    Ogre::Vector3 c = Ogre::Vector3::ZERO;
    double c_dot_current_point = -1.0;  // Initializing to -1 means the first point is always chosen.

    for (const Ogre::Vector3& point : points) {
      if (point == current_point) {
        continue;
      }

      if (c.dotProduct(point) > c_dot_current_point) {
        candidate_point = point;

        // Just making it explicitly clear we are only working with convexifying in the xy plane
        Ogre::Vector3 point_minus_current_point((candidate_point.x - current_point.x), (candidate_point.y - current_point.y), 0.0);

        // Rotate 90 degrees to point 'leftwards'
        c.x = -point_minus_current_point.y;
        c.y = point_minus_current_point.x;

        c_dot_current_point = c.dotProduct(current_point);
      }
    }

    // If we've found any other points
    if (c != Ogre::Vector3::ZERO) {
      vertices.push_back(candidate_point);
    }

  } while (vertices.front() != vertices.back());

  return vertices;
}

}  // namespace anymal_rviz_plugin
