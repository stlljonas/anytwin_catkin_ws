/*!
 * @file     ComSupportControlLeverConfiguration.hpp
 * @author   C. Dario Bellicoso
 * @date     Oct 7, 2014
 * @brief
 */
#include "loco/torso_control/ComSupportControlLeverConfiguration.hpp"
#include "loco/common/TerrainModelFreePlane.hpp"

namespace loco {

ComSupportControlLeverConfiguration::ComSupportControlLeverConfiguration(WholeBody& wholeBody, TerrainModelBase& terrainModel)
    : ComSupportControlBase(*wholeBody.getLegsPtr()), torso_(*wholeBody.getTorsoPtr()), terrainModel_(terrainModel) {
  positionCenterToForeHindSupportFeetInControlFrame_[0] = Position::Zero();
  positionCenterToForeHindSupportFeetInControlFrame_[1] = Position::Zero();
  positionWorldToCenterInWorldFrame_ = Position::Zero();
}

bool ComSupportControlLeverConfiguration::initialize(double dt) {
  return true;
}

bool ComSupportControlLeverConfiguration::setToInterpolated(const ComSupportControlBase& supportPolygon1,
                                                            const ComSupportControlBase& supportPolygon2, double t) {
  return false;
}

bool ComSupportControlLeverConfiguration::advance(double dt) {
  Position comTarget;

  // Matlab code
  //  % Find ratios - control vertical
  //  ratio_x_controlvertical = abs((rb_onplane_worldvertical(1) - rf_lf_controlvertical(1))/...
  //                            (rb_onplane_worldvertical(1) - rf_lh_controlvertical(1)));
  //  ratio_y_controlvertical = abs((rb_onplane_worldvertical(2) - rf_lf_controlvertical(2))/...
  //                            (rb_onplane_worldvertical(2) - rf_rf_controlvertical(2)));
  //
  //  shift_y = hipToHipY*ratio_x_controlvertical/(1+ratio_x_controlvertical) - rf_lf_controlvertical(2);

  RotationQuaternion orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();
  RotationQuaternion orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();

  auto& terrainModel = dynamic_cast<TerrainModelFreePlane&>(terrainModel_);

  Position rb_onplane_worldvertical = torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
  terrainModel.getHeight(rb_onplane_worldvertical);

  Position rb_onplane_controlvertical = terrainModel.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(
      torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame());
  //  std::cout << "rb onplane control: " << rb_onplane_controlvertical << std::endl;
  //  std::cout << "rb onplane world: " << rb_onplane_worldvertical << std::endl;

  positionWorldToCenterInWorldFrame_ = rb_onplane_controlvertical;

  Position positionBaseOnTerrainToForeSupportLegInControlFrame;
  Position positionBaseOnTerrainToHindSupportLegInControlFrame;

  // Get contact points
  // fixme: fix get(id) access
  if ((legs_.get(0).getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
      (legs_.get(0).getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
    positionBaseOnTerrainToForeSupportLegInControlFrame = terrainModel.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(
        legs_.get(0).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
    positionBaseOnTerrainToForeSupportLegInControlFrame =
        orientationWorldToControl.rotate(positionBaseOnTerrainToForeSupportLegInControlFrame - rb_onplane_controlvertical);

    positionBaseOnTerrainToHindSupportLegInControlFrame = terrainModel.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(
        legs_.get(3).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
    positionBaseOnTerrainToHindSupportLegInControlFrame =
        orientationWorldToControl.rotate(positionBaseOnTerrainToHindSupportLegInControlFrame - rb_onplane_controlvertical);

    positionCenterToForeHindSupportFeetInControlFrame_[0] = positionBaseOnTerrainToForeSupportLegInControlFrame;
    positionCenterToForeHindSupportFeetInControlFrame_[1] = positionBaseOnTerrainToHindSupportLegInControlFrame;
  } else {
    positionBaseOnTerrainToForeSupportLegInControlFrame = terrainModel.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(
        legs_.get(1).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
    positionBaseOnTerrainToForeSupportLegInControlFrame =
        orientationWorldToControl.rotate(positionBaseOnTerrainToForeSupportLegInControlFrame - rb_onplane_controlvertical);

    positionBaseOnTerrainToHindSupportLegInControlFrame = terrainModel.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(
        legs_.get(2).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
    positionBaseOnTerrainToHindSupportLegInControlFrame = orientationWorldToControl.rotate(
        legs_.get(2).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() - rb_onplane_controlvertical);

    positionCenterToForeHindSupportFeetInControlFrame_[0] = positionBaseOnTerrainToForeSupportLegInControlFrame;
    positionCenterToForeHindSupportFeetInControlFrame_[1] = positionBaseOnTerrainToHindSupportLegInControlFrame;
  }

  Position r_controlToBaseOnTerrainInControlFrame = orientationWorldToControl.rotate(rb_onplane_worldvertical - rb_onplane_controlvertical);

  double ratioX = fabs(r_controlToBaseOnTerrainInControlFrame.x() - positionBaseOnTerrainToForeSupportLegInControlFrame.x()) /
                  fabs(r_controlToBaseOnTerrainInControlFrame.x() - positionBaseOnTerrainToHindSupportLegInControlFrame.x());

  double hipToHipX =
      fabs(positionBaseOnTerrainToForeSupportLegInControlFrame.x() - positionBaseOnTerrainToHindSupportLegInControlFrame.x());
  double hipToHipY =
      fabs(positionBaseOnTerrainToForeSupportLegInControlFrame.y() - positionBaseOnTerrainToHindSupportLegInControlFrame.y());

  comTarget = r_controlToBaseOnTerrainInControlFrame;

  double ly_2 = 0.0;
  double ratioY = 0.0;

  if ((legs_.get(0).getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
      (legs_.get(0).getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
    ly_2 = hipToHipY / (1.0 + ratioX);
    comTarget.y() = -(hipToHipY * 0.5 - ly_2);
  } else {
    ly_2 = hipToHipY * ratioX / (1.0 + ratioX);
    comTarget.y() = ly_2 - hipToHipY * 0.5;
  }

  Position aux = rb_onplane_controlvertical + orientationWorldToControl.inverseRotate(comTarget);
  comTarget = aux;

  ratioY = fabs(comTarget.y() - positionBaseOnTerrainToForeSupportLegInControlFrame.y()) /
           fabs(comTarget.y() - positionBaseOnTerrainToHindSupportLegInControlFrame.y());

  //  std::cout << "check ratios" << std::endl
  //            << "x: " << ratioX << std::endl
  //            << "y: " << ratioY << std::endl
  //            << "ly2/ly1: " << ly_2/(hipToHipY-ly_2) << std::endl
  //            << "ly1/ly2: " << 1.0/(ly_2/(hipToHipY-ly_2)) << std::endl
  //            << "com: " << comTarget << std::endl
  //            << "fore support" << positionBaseOnTerrainToForeSupportLegInControlFrame << std::endl;

  positionWorldToDesiredCoMInWorldFrame_ = comTarget + Position(headingOffset_, lateralOffset_, 0.0);
  positionWorldToDesiredCoMInWorldFrame_.z() = 0.0;

  return true;
}

} /* namespace loco */
