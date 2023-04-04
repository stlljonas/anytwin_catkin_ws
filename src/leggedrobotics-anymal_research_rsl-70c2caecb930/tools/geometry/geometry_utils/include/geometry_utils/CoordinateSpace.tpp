/**
 * @authors     Marco Tranzatto, Gabriel Hottiger
 * @affiliation RSL, ANYbotics
 * @brief       Coordinate space calculations.
 */

#pragma once

#include "geometry_utils/CoordinateSpace.hpp"
#include "geometry_utils/helper_methods.hpp"
#include "message_logger/message_logger.hpp"

namespace geometry_utils {

template <TranslationDimensionType Dim>
double CoordinateSpace::getWeightedDistanceBetweenPoints(const Vector3D& point1, const Vector3D& point2, const Vector3D& weights) {
  return getWeightedDifferenceBetweenPoints<Dim>(point1, point2, weights).norm();
}

template <TranslationDimensionType Dim>
bool CoordinateSpace::isPointOnLineSegment(const Vector3D& point, const Vector3D& startPoint, const Vector3D& endPoint, double tolerance) {
  Vector3D r1 = getDifferenceBetweenPoints<Dim>(startPoint, point);
  Vector3D r2 = getDifferenceBetweenPoints<Dim>(startPoint, endPoint);

  // point lies on line
  if (r1.cross(r2.normalized()).norm() > tolerance) {
    return false;
  }

  // not behind start (tolerance circle around start point)
  double dotProduct = r1.dot(r2);
  if (dotProduct < 0) {
    return r1.norm() < tolerance;
  }

  // not behind end (tolerance circle around end point)
  const double r2Norm = r2.norm();
  if (dotProduct > (r2Norm * r2Norm)) {
    return getDistanceBetweenPoints<Dim>(endPoint, point) < tolerance;
  }

  return true;
}

template <TranslationDimensionType Dim>
CoordinateSpace::ClosestPointOnLineSegmentInfo CoordinateSpace::calculateWeightedClosestPointInfoOnLineSegment(
    const Vector3D& referencePoint, const Vector3D& startPoint, const Vector3D& endPoint, const Vector3D& weights) {
  ClosestPointOnLineSegmentInfo closestPointInfo;
  // Calculate distance to startPoint if the line is negligible
  const double lineLength = CoordinateSpace::getWeightedDistanceBetweenPoints<Dim>(startPoint, endPoint, weights);
  if (almostEqual(lineLength, 0.0)) {
    closestPointInfo.point_ = startPoint;
    closestPointInfo.distance_ = CoordinateSpace::getWeightedDistanceBetweenPoints<Dim>(startPoint, referencePoint, weights);
    closestPointInfo.isCloserToStart_ = true;
  } else {
    // use only 3D position to compute closest point.
    Vector3D v_0_1{getWeightedDifferenceBetweenPoints<Dim>(startPoint, endPoint, weights)};
    Vector3D v_0_ref{getWeightedDifferenceBetweenPoints<Dim>(startPoint, referencePoint, weights)};
    double normalizedProjectionOnLine = (v_0_1.dot(v_0_ref)) / (v_0_1.dot(v_0_1));
    if (normalizedProjectionOnLine <= 0.0) {
      closestPointInfo.point_ = startPoint;
      closestPointInfo.isCloserToStart_ = true;
    } else if (normalizedProjectionOnLine >= 1.0) {
      closestPointInfo.point_ = endPoint;
      closestPointInfo.isCloserToStart_ = false;
    } else {
      // convex combination but do not consider orientation.
      closestPointInfo.point_ = normalizedProjectionOnLine * endPoint + (1.0 - normalizedProjectionOnLine) * startPoint;
      closestPointInfo.isCloserToStart_ = (normalizedProjectionOnLine <= 0.5);
    }
    closestPointInfo.distance_ = CoordinateSpace::getWeightedDistanceBetweenPoints<Dim>(referencePoint, closestPointInfo.point_, weights);
  }
  return closestPointInfo;
}

template <DimensionType Dim>
CoordinateSpace::ClosestPoseOnLineSegmentInfo CoordinateSpace::calculateWeightedClosestPoseInfoOnLineSegment(
    const PoseStamped& referencePose, const PoseStamped& startPose, const PoseStamped& endPose, const Vector4D& weights) {
  // Calculate distance to startPoint if the line is negligible
  const double lineLength = CoordinateSpace::getWeightedDistanceBetweenPoses<Dim>(startPose, endPose, weights);
  if (almostEqual(lineLength, 0.0)) {
    ClosestPoseOnLineSegmentInfo closestPoseInfo;
    closestPoseInfo.success_ = true;
    closestPoseInfo.pose_ = startPose;
    closestPoseInfo.distance_ = CoordinateSpace::getWeightedDistanceBetweenPoses<Dim>(startPose, referencePose, weights);
    closestPoseInfo.isCloserToStart_ = true;
    return closestPoseInfo;
  }

  /**
   * We are trying to find the closest point analytically. The yaw distance has a wrap around and therefore the dot product yields wrong
   * output. That is why we calculate two dot products: one using the end point as reference point and one using the start point as
   * reference point. The one with smaller total distance yields the closest pose.
   */

  // Calculate the dot product using the start as a reference point.
  ClosestPoseOnLineSegmentInfo closestPoseInfoFromStart;
  closestPoseInfoFromStart.success_ = true;

  Vector4D v_0_1 = getWeightedDifferenceBetweenPoses<Dim>(startPose, endPose, weights);
  Vector4D v_0_ref = getWeightedDifferenceBetweenPoses<Dim>(startPose, referencePose, weights);

  double normalizedProjectionOnLineFromStart = (v_0_1.dot(v_0_ref)) / (v_0_1.dot(v_0_1));

  if (normalizedProjectionOnLineFromStart <= 0.0) {
    closestPoseInfoFromStart.pose_ = startPose;
    closestPoseInfoFromStart.isCloserToStart_ = true;
  } else if (normalizedProjectionOnLineFromStart >= 1.0) {
    closestPoseInfoFromStart.pose_ = endPose;
    closestPoseInfoFromStart.isCloserToStart_ = false;
  } else {
    closestPoseInfoFromStart.pose_ = startPose;
    closestPoseInfoFromStart.success_ = closestPoseInfoFromStart.pose_.interpolate(startPose, endPose, normalizedProjectionOnLineFromStart);
    closestPoseInfoFromStart.isCloserToStart_ = (normalizedProjectionOnLineFromStart <= 0.5);
  }
  closestPoseInfoFromStart.distance_ =
      CoordinateSpace::getWeightedDistanceBetweenPoses<Dim>(referencePose, closestPoseInfoFromStart.pose_, weights);

  // Calculate the dot product using the end as a reference point.
  ClosestPoseOnLineSegmentInfo closestPoseInfoFromEnd;
  closestPoseInfoFromEnd.success_ = true;

  Vector4D v_1_0 = getWeightedDifferenceBetweenPoses<Dim>(endPose, startPose, weights);
  Vector4D v_1_ref = getWeightedDifferenceBetweenPoses<Dim>(endPose, referencePose, weights);

  double normalizedProjectionOnLineFromEnd = (v_1_0.dot(v_1_ref)) / (v_1_0.dot(v_1_0));

  if (normalizedProjectionOnLineFromEnd <= 0.0) {
    closestPoseInfoFromEnd.pose_ = endPose;
    closestPoseInfoFromEnd.isCloserToStart_ = false;
  } else if (normalizedProjectionOnLineFromEnd >= 1.0) {
    closestPoseInfoFromEnd.pose_ = startPose;
    closestPoseInfoFromEnd.isCloserToStart_ = true;
  } else {
    closestPoseInfoFromEnd.pose_ = endPose;
    closestPoseInfoFromEnd.success_ = closestPoseInfoFromEnd.pose_.interpolate(endPose, startPose, normalizedProjectionOnLineFromEnd);
    closestPoseInfoFromEnd.isCloserToStart_ = (normalizedProjectionOnLineFromEnd >= 0.5);
  }
  closestPoseInfoFromEnd.distance_ =
      CoordinateSpace::getWeightedDistanceBetweenPoses<Dim>(referencePose, closestPoseInfoFromEnd.pose_, weights);

  // Return the pose of the solution candidates that is closer.
  if (closestPoseInfoFromEnd.distance_ < closestPoseInfoFromStart.distance_) {
    return closestPoseInfoFromEnd;
  }

  return closestPoseInfoFromStart;
}

}  // namespace geometry_utils
