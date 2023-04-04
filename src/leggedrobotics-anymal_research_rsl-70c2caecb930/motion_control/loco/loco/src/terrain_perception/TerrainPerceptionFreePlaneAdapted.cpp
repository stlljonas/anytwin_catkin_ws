/*
 * TerrainPerceptionFreePlaneAdapted.cpp
 *
 *  Created on: Jul 11, 2017
 *      Author: dbellicoso
 */

#include <loco/terrain_perception/TerrainPerceptionFreePlaneAdapted.hpp>

namespace loco {

TerrainPerceptionFreePlaneAdapted::TerrainPerceptionFreePlaneAdapted(TerrainModelPlane& terrainModel, WholeBody& wholeBody,
                                                                     HeadingGenerator& headingGenerator,
                                                                     TerrainPerceptionFreePlane::EstimatePlaneInFrame estimatePlaneInFrame,
                                                                     ControlFrameHeading referenceHeading)
    : Base(terrainModel, wholeBody, headingGenerator, estimatePlaneInFrame, referenceHeading) {}

void TerrainPerceptionFreePlaneAdapted::updatePlaneEstimation() {
  /* estimate the plane which best fits the most recent contact points of each foot in world frame
   * using least squares (pseudo inversion of the regressor matrix H)
   *
   * parameters       -> [a b d]^T
   * plane equation   -> z = d-ax-by
   * normal to plane  -> n = [a b 1]^T
   *
   * */
  Eigen::MatrixXd linearRegressor = Eigen::MatrixXd::Zero(4, 3);
  Eigen::Vector4d measuredFootHeights = Eigen::Vector4d::Zero();

  std::vector<loco::Position> mostRecenPositionOfFootInWorldFrame(legs_.size(), loco::Position());

  for (auto leg : legs_) {
    const auto legId = leg->getId();

    // Take location of stance feet and desired foothold of swing feet.
    mostRecenPositionOfFootInWorldFrame[legId] = mostRecentPositionOfFoot_[legId];
    if (estimatePlaneInFrame_ == EstimatePlaneInFrame::Base) {
      homogeneousTransformFromBaseToWorldFrame(mostRecenPositionOfFootInWorldFrame[legId], legId);
    }

    if (!leg->getContactSchedule().isAndShouldBeGrounded()) {
      double swingPhase = leg->getContactSchedule().getSwingPhase();
      if (swingPhase == -1.0) {
        swingPhase = 1.0;
      }

      const loco::Position& footHoldInWorldFrame = leg->getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame();
      mostRecenPositionOfFootInWorldFrame[legId] =
          (1.0 - swingPhase) * mostRecenPositionOfFootInWorldFrame[legId] + swingPhase * footHoldInWorldFrame;
    }
  }

  linearRegressor << -mostRecenPositionOfFootInWorldFrame[0].x(), -mostRecenPositionOfFootInWorldFrame[0].y(), 1.0,
      -mostRecenPositionOfFootInWorldFrame[1].x(), -mostRecenPositionOfFootInWorldFrame[1].y(), 1.0,
      -mostRecenPositionOfFootInWorldFrame[2].x(), -mostRecenPositionOfFootInWorldFrame[2].y(), 1.0,
      -mostRecenPositionOfFootInWorldFrame[3].x(), -mostRecenPositionOfFootInWorldFrame[3].y(), 1.0;
  measuredFootHeights << mostRecenPositionOfFootInWorldFrame[0].z(), mostRecenPositionOfFootInWorldFrame[1].z(),
      mostRecenPositionOfFootInWorldFrame[2].z(), mostRecenPositionOfFootInWorldFrame[3].z();

  // Solve least squares problem.
  const Eigen::Vector3d parameters = linearRegressor.colPivHouseholderQr().solve(measuredFootHeights);

  /* Find a point on the plane. From z = d-ax-by, it is easy to find that p = [0 0 d]
   * is on the plane
   */
  positionInWorldFrameFilterInput_ << 0.0, 0.0, parameters(2);

  /* From the assumption that the normal has always unit z-component,
   * its norm will always be greater than zero
   */
  normalInWorldFrameFilterInput_ << parameters(0), parameters(1), 1.0;
}  // update plane estimation

} /* namespace loco */
