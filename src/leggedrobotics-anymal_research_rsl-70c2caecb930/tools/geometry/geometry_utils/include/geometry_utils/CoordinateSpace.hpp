/**
 * @authors     Remo Diethelm, Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Coordinate space calculations.
 */

#pragma once

#include "geometry_utils/DimensionType.hpp"
#include "geometry_utils/PoseStamped.hpp"

namespace geometry_utils {

class CoordinateSpace {
 public:
  //! Struct to hold output of closest point to line call
  struct ClosestPointOnLineSegmentInfo {
    //! Point on the segment which is closest to reference point.
    Vector3D point_ = Vector3D::Zero();
    //! Minimum distance from reference point to closest point on the line.
    double distance_ = std::numeric_limits<double>::max();
    //! True if computed point is closer to start point, false if closer to end point of the line segment
    bool isCloserToStart_ = false;
  };

  //! Struct to hold output of closest point to line call
  struct ClosestPoseOnLineSegmentInfo {
    //! Frames can be different and pose can fail
    bool success_ = false;
    //! Point on the segment which is closest to reference point.
    PoseStamped pose_{};
    //! Minimum distance from reference point to closest point on the line.
    double distance_ = std::numeric_limits<double>::max();
    //! True if computed point is closer to start point, false if closer to end point of the line segment
    bool isCloserToStart_ = false;
  };

  /**
   * Get the weighted difference between two points.
   * @param point1 First point.
   * @param point2 Second point.
   * @param weights Weight vector [w_x, w_y, w_z] used to scale the distance.
   * @return Difference between the points.
   */
  template <TranslationDimensionType Dim>
  static Vector3D getWeightedDifferenceBetweenPoints(const Vector3D& point1, const Vector3D& point2, const Vector3D& weights);

  /**
   * Get the difference between two points.
   * @param point1 First point.
   * @param point2 Second point.
   * @return Difference between the points.
   */
  template <TranslationDimensionType Dim>
  static Vector3D getDifferenceBetweenPoints(const Vector3D& point1, const Vector3D& point2) {
    return getWeightedDifferenceBetweenPoints<Dim>(point1, point2, Vector3D::Ones());
  }

  /**
   * Get the weighted distance between two points.
   * @param point1 First point.
   * @param point2 Second point.
   * @param weights Weight vector [w_x, w_y, w_z] used to scale the distance.
   * @return Distance between the points.
   */
  template <TranslationDimensionType Dim>
  static double getWeightedDistanceBetweenPoints(const Vector3D& point1, const Vector3D& point2, const Vector3D& weights);

  /**
   * Get the distance between two points.
   * @param point1 First point.
   * @param point2 Second point.
   * @return Distance between the points.
   */
  template <TranslationDimensionType Dim>
  static double getDistanceBetweenPoints(const Vector3D& point1, const Vector3D& point2) {
    return getWeightedDistanceBetweenPoints<Dim>(point1, point2, Vector3D::Ones());
  }

  /*!
   * Get the weighted distance between two poses in the given dimension.
   * @param[in] pose1 pose 1.
   * @param[in] pose2 pose 2.
   * @param[in] weights Scale distances with weights = [w_x, w_y, w_z, w_rotation]. The translational weights have dimension
   * [1/m] and the rotational weight [1/rad] (default 1m = 1rad).
   * @return Distance in given dimension.
   */
  template <enum DimensionType Dim>
  static double getWeightedDistanceBetweenPoses(const PoseStamped& pose1, const PoseStamped& pose2, const Vector4D& weights);

  /*!
   * Get the distance between two poses in the given dimension.
   * @param[in] pose1 pose 1.
   * @param[in] pose2 pose 2.
   * @return distance.
   */
  template <enum DimensionType Dim>
  static double getDistanceBetweenPoses(const PoseStamped& pose1, const PoseStamped& pose2) {
    return getWeightedDistanceBetweenPoses<Dim>(pose1, pose2, Vector4D::Ones());
  }

  /*!
   * Wrap an angle into (-pi,pi].
   * @param[in] angle input angle.
   * @return wrapped angle.
   */
  static double wrapAngle(double angle);

  /*!
   * Get the difference between two angles within [-Pi..Pi].
   * @param[in] angle1 angle 1.
   * @param[in] angle2 angle 2.
   * @return difference.
   */
  static double getDifferenceBetweenAngles(double angle1, double angle2);

  /*!
   * Get the distance between two angles within [0..Pi].
   * @param[in] angle1 angle 1.
   * @param[in] angle2 angle 2.
   * @return distance.
   */
  static double getDistanceBetweenAngles(double angle1, double angle2);

  /**
   * Checks if a point lies on the line segment from startPoint to endPoint.
   * @param point reference point.
   * @param startPoint line segment start point.
   * @param endPoint line segment end point.
   * @param tolerance distance tolerance. Default 1mm. (margin around line, perpendicular distance along line, circle around start and end)
   * @return true, if point lies on the line segment.
   */
  template <TranslationDimensionType Dim>
  static bool isPointOnLineSegment(const Vector3D& point, const Vector3D& startPoint, const Vector3D& endPoint, double tolerance = 1e-3);

  /*!
   * Compute closest point on a line segment with respect to a given point, in the given weighted translational dimension.
   * @param referencePoint reference point.
   * @param startPoint line segment start point.
   * @param endPoint line segment end point.
   * @param weights Weight vector [w_x, w_y, w_z] used to scale the distance.
   * @return Information about to closest point on the line.
   */
  template <TranslationDimensionType Dim>
  static ClosestPointOnLineSegmentInfo calculateWeightedClosestPointInfoOnLineSegment(const Vector3D& referencePoint,
                                                                                      const Vector3D& startPoint, const Vector3D& endPoint,
                                                                                      const Vector3D& weights);

  /*!
   * Compute closest point on a line segment with respect to a given point, in the given translational dimension.
   * @param referencePoint reference point.
   * @param startPoint line segment start point.
   * @param endPoint line segment end point.
   * @return Information about to closest point on the line.
   */
  template <TranslationDimensionType Dim>
  static ClosestPointOnLineSegmentInfo calculateClosestPointInfoOnLineSegment(const Vector3D& referencePoint, const Vector3D& startPoint,
                                                                              const Vector3D& endPoint) {
    return calculateWeightedClosestPointInfoOnLineSegment<Dim>(referencePoint, startPoint, endPoint, Vector3D::Ones());
  }

  /*!
   * Compute closest pose on a line segment with respect to a given point, in the given translational dimension.
   * @param referencePoint reference point.
   * @param startPose line segment start pose.
   * @param endPose line segment end pose.
   * @param[in] weights Scale distances with weights = [w_x, w_y, w_z, w_rotation]. The translational weights have dimension
   * @return Information about to closest pose on the line. Orientation interpolated between start and end.
   */
  template <DimensionType Dim>
  static ClosestPoseOnLineSegmentInfo calculateWeightedClosestPoseInfoOnLineSegment(const PoseStamped& referencePose,
                                                                                    const PoseStamped& startPose,
                                                                                    const PoseStamped& endPose, const Vector4D& weights);

  /*!
   * Compute closest pose on a line segment with respect to a given point, in the given translational dimension.
   * @param referencePoint reference point.
   * @param startPose line segment start pose.
   * @param endPose line segment end pose.
   * @return Information about to closest pose on the line. Orientation interpolated between start and end.
   */
  template <DimensionType Dim>
  static ClosestPoseOnLineSegmentInfo calculateClosestPoseInfoOnLineSegment(const PoseStamped& referencePose, const PoseStamped& startPose,
                                                                            const PoseStamped& endPose) {
    return calculateWeightedClosestPoseInfoOnLineSegment<Dim>(referencePose, startPose, endPose, Vector4D::Ones());
  }

 private:
  enum Vector4Index { I_X = 0, I_Y = 1, I_Z = 2, I_ROT = 3 };

  /**
   * Get the weighted difference between two poses.
   * @param pose1 First pose.
   * @param pose2 Second pose.
   * @param[in] weights Scale distances with weights = [w_x, w_y, w_z, w_rotation]. The translational weights have dimension
   * @return Difference between the poses.
   */
  template <DimensionType Dim>
  static Vector4D getWeightedDifferenceBetweenPoses(const PoseStamped& pose1, const PoseStamped& pose2, const Vector4D& weights);
};

}  // namespace geometry_utils

#include "geometry_utils/CoordinateSpace.tpp"