
/*!
 * @file    MapLocalizationTsif.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

#pragma once

#include <tsif/filter_with_definition.h>
#include <anymal_state_estimator_tsif/tsif/MapLocalizationTsifDefinition.hpp>

#include <kindr/Core>

#include <any_node/Param.hpp>
#include <message_logger/message_logger.hpp>

namespace tsif {
//localization filter class
class MapLocalizationTsif : public FilterWithDefinition<MapLocalizationTsifDefinition> {
 public:
  using Base = FilterWithDefinition<MapLocalizationTsifDefinition>;

  using Base::FD;

  explicit MapLocalizationTsif(const ros::NodeHandle& node_handle) {
    const auto parameter_prefix = std::string{"MapLocalizationTsif"};
    //get filter parameters
    max_iter_ = param_io::param<int>(node_handle, parameter_prefix+std::string{"/filter/max_iter"}, 1);
    th_iter_ = param_io::param<double>(node_handle, parameter_prefix+std::string{"/filter/th_iter"}, 0.1);
    //get parameters for the residuals
    std::get<FD::ResidualEnum::BASE_POSE_UPD>(residuals_)
        .LoadParameters(node_handle, parameter_prefix+std::string{"/residuals/base_pose_update"});
    std::get<FD::ResidualEnum::BASE_POSE_UPD>(timelines_)
        .SetMaxWaitTime(param_io::param<double>(node_handle,
                                                parameter_prefix+std::string{"/timelines/base_pose_update/max_wait_time"},
                                                0.1));

    std::get<FD::ResidualEnum::MAP_POSE_UPD>(residuals_)
        .LoadParameters(node_handle, parameter_prefix+std::string{"/residuals/icp_pose_update"});
    std::get<FD::ResidualEnum::MAP_POSE_UPD>(timelines_)
        .SetMaxWaitTime(param_io::param<double>(node_handle,
                                                parameter_prefix+std::string{"/timelines/icp_pose_update/max_wait_time"},
                                                0.1));

    std::get<FD::ResidualEnum::BASE_POSE_PRD>(residuals_)
        .LoadParameters(node_handle, parameter_prefix+std::string{"/residuals/base_pose_prediction"});
    std::get<FD::ResidualEnum::BASE_POSE_PRD>(timelines_)
        .SetMaxWaitTime(param_io::param<double>(node_handle,
                                                parameter_prefix+std::string{"/timelines/base_pose_prediction/max_wait_time"},
                                                0.1));

    std::get<FD::ResidualEnum::MAP_POSE_PRD>(residuals_)
        .LoadParameters(node_handle, parameter_prefix+std::string{"/residuals/icp_pose_prediction"});
    std::get<FD::ResidualEnum::MAP_POSE_PRD>(timelines_)
        .SetMaxWaitTime(param_io::param<double>(node_handle,
                                                parameter_prefix+std::string{"/timelines/icp_pose_prediction/max_wait_time"},
                                                0.1));

    //get parameters for the states
    I_diag_init_.at(FD::StateEnum::I_R_IB) =
        param_io::param<Vec3>(node_handle, parameter_prefix+std::string{"/states/I_r_IB/information"}, Vec3(1., 1., 1.));
    I_diag_init_.at(FD::StateEnum::PHI_IB) =
        param_io::param<Vec3>(node_handle, parameter_prefix+std::string{"/states/Phi_IB/information"}, Vec3(1., 1., 1.));
    I_diag_init_.at(FD::StateEnum::I_R_IJ) =
        param_io::param<Vec3>(node_handle, parameter_prefix+std::string{"/states/I_r_IJ/information"}, Vec3(1., 1., 1.));
    I_diag_init_.at(FD::StateEnum::PHI_IJ) =
        param_io::param<Vec3>(node_handle, parameter_prefix+std::string{"/states/Phi_IJ/information"}, Vec3(1., 1., 1.));
    //set remaining parameters to defaults
    T_IB_init_ = kindr::HomTransformQuatD(kindr::Position3D(0.0, 0.0, 0.0), kindr::RotationQuaternionD(1.,0.,0.,0.));
    //set remaining parameters to defaults
    T_JV_init_ = kindr::HomTransformQuatD(kindr::Position3D(0.0, 0.0, 0.0), kindr::RotationQuaternionD(1.,0.,0.,0.));
    T_BV_ = kindr::HomTransformQuatD(kindr::Position3D(0.0, 0.0, 0.0), kindr::RotationQuaternionD(1.,0.,0.,0.));
  }

  ~MapLocalizationTsif() override = default;

  //initialization function, called during the first uninitialized update of the TSIF
  void Init(TimePoint t) override {
    if(GetMinMaxTime() != TimePoint::min()){
      startTime_ = t;
      time_ = t;
      //set initial state
      state_.Get<FD::StateEnum::I_R_IB>() = T_IB_init_.getPosition().toImplementation();
      state_.Get<FD::StateEnum::PHI_IB>() = T_IB_init_.getRotation().toImplementation().normalized();
      state_.Get<FD::StateEnum::PHI_IJ>() =
          state_.Get<FD::StateEnum::PHI_IB>() * T_JV_init_.getRotation().toImplementation().normalized().inverse();
      state_.Get<FD::StateEnum::I_R_IJ>() =
          state_.Get<FD::StateEnum::I_R_IB>()
          - state_.Get<FD::StateEnum::PHI_IJ>().toRotationMatrix()*T_JV_init_.getPosition().toImplementation();
      //set unoptimized states (parameters)
      state_.Get<FD::ParamEnum::B_R_BV>() = T_BV_.getPosition().toImplementation();
      state_.Get<FD::ParamEnum::PHI_BV>() = T_BV_.getRotation().toImplementation().normalized();
      state_.Get<FD::ParamEnum::R_NULL>() = Vec3::Zero();
      state_.Get<FD::ParamEnum::PHI_NULL>() = Quat::Identity();

      //set initial information matrix
      I_.setIdentity();
      for (int i=0; i<3; i++){
        I_.block<3,3>(State::Start(FD::StateEnum::I_R_IB),State::Start(FD::StateEnum::I_R_IB))(i,i) = I_diag_init_.at(FD::StateEnum::I_R_IB)(i);
        I_.block<3,3>(State::Start(FD::StateEnum::PHI_IB),State::Start(FD::StateEnum::PHI_IB))(i,i) = I_diag_init_.at(FD::StateEnum::PHI_IB)(i);
        I_.block<3,3>(State::Start(FD::StateEnum::I_R_IJ),State::Start(FD::StateEnum::I_R_IJ))(i,i) = I_diag_init_.at(FD::StateEnum::I_R_IJ)(i);
        I_.block<3,3>(State::Start(FD::StateEnum::PHI_IJ),State::Start(FD::StateEnum::PHI_IJ))(i,i) = I_diag_init_.at(FD::StateEnum::PHI_IJ)(i);
      }
      is_initialized_ = true;
    }
  }

  TimePoint GetLastIcpStamp() const { return std::get<FD::ResidualEnum::MAP_POSE_UPD>(timelines_).GetLastTime(); }
  void SetInitOdomPose(const kindr::HomTransformQuatD& T_IB_init) { T_IB_init_ = T_IB_init; }
  void SetInitIcpPose(const kindr::HomTransformQuatD& T_JV_init) { T_JV_init_ = T_JV_init; }
  void SetPoseMeasurementFrameOffset(const kindr::HomTransformQuatD& T_BV) { T_BV_ = T_BV; }

  void AddMeasurementPoseBaseToWorld(const tsif::TimePoint& time, const Vec3& I_r_IB, const Quat& q_IB){
    AddMeas<FD::ResidualEnum::BASE_POSE_UPD>(time, std::make_shared<tsif::MeasPoseExtFrameCentric>(I_r_IB, q_IB));
  }
  void AddMeasurementPoseMeasurementFrameToMap(const tsif::TimePoint& time, const Vec3& J_r_JV, const Quat& q_JV){
    AddMeas<FD::ResidualEnum::MAP_POSE_UPD>(time, std::make_shared<tsif::MeasPoseOdomFrameCentric>(J_r_JV, q_JV));
  }

  tsif::Quat GetOrientationBaseToMap(){ return state_.Get<FD::StateEnum::PHI_IJ>().inverse() * state_.Get<FD::StateEnum::PHI_IB>(); }
  tsif::Vec3 GetPositionMapToBaseInMap(){
    return state_.Get<FD::StateEnum::PHI_IJ>().inverse().toRotationMatrix() * (state_.Get<FD::StateEnum::I_R_IB>() - state_.Get<FD::StateEnum::I_R_IJ>());
  }
  tsif::Quat GetOrientationMapToWorld(){ return state_.Get<FD::StateEnum::PHI_IJ>(); }
  tsif::Vec3 GetPositionWorldToMapInWorld(){ return state_.Get<FD::StateEnum::I_R_IJ>(); };

 protected:
  //initial information diagonals
  std::array<Vec3,FD::StateEnum::NUM_STATES> I_diag_init_;
  //initial pose base to odom
  kindr::HomTransformQuatD T_IB_init_;
  //initial pose measurement frame to map
  kindr::HomTransformQuatD T_JV_init_;
  //pose measurement frame to base
  kindr::HomTransformQuatD T_BV_;
};

} /* namespace tsif */