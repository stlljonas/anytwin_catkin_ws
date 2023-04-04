
/*!
 * @file    ImuKinAnymalOdometryTsif.hpp
 * @author  Fabian Tresoldi
 * @date    May, 2017
 */

#pragma once

#include <anymal_state_estimator_tsif/tsif/ImuKinAnymalOdometryTsifDefinition.hpp>
#include <anymal_state_estimator_tsif/tsif/AnymalOdometryTsif.hpp>

#include <kindr/Core>

#include <any_node/Param.hpp>
#include <message_logger/message_logger.hpp>

#include <anymal_description/AnymalDescription.hpp>
#include <anymal_model/AnymalModel.hpp>

namespace tsif {

using ImuKinAnymalOdometryTsifBase = AnymalOdometryTsif<anymal_description::ConcreteAnymalDescription,
                                                        anymal_model::AnymalState,
                                                        ImuKinAnymalOdometryTsifDefinition>;

//odometry filter class for anymal fusing imu measurements and kinematics
class ImuKinAnymalOdometryTsif : public ImuKinAnymalOdometryTsifBase {
 public:
  using Base = ImuKinAnymalOdometryTsifBase;
  using Base::FD;
  using Base::RD;

  //compile time maps between the tsif indices and anymal model
  template <RD::ContactEnum contact, FD::ResidualEnum residual>
  using contactToResidualKV = std_utils::KeyValuePair<RD::ContactEnum, FD::ResidualEnum, contact, residual>;
  template <RD::ContactEnum contact, FD::StateEnum contactState>
  using contactToContactStateKV = std_utils::KeyValuePair<RD::ContactEnum, FD::StateEnum, contact, contactState>;

  using mapContactToLandmarkPrediction =
      std_utils::CompileTimeMap<RD::ContactEnum, FD::ResidualEnum,
                                contactToResidualKV<RD::ContactEnum::LF_FOOT, FD::ResidualEnum::LMK_PRD_LF>,
                                contactToResidualKV<RD::ContactEnum::RF_FOOT, FD::ResidualEnum::LMK_PRD_RF>,
                                contactToResidualKV<RD::ContactEnum::LH_FOOT, FD::ResidualEnum::LMK_PRD_LH>,
                                contactToResidualKV<RD::ContactEnum::RH_FOOT, FD::ResidualEnum::LMK_PRD_RH>>;

  using mapContactToContactState =
      std_utils::CompileTimeMap<RD::ContactEnum, FD::StateEnum,
                                contactToContactStateKV<RD::ContactEnum::LF_FOOT, FD::StateEnum::I_P_LF>,
                                contactToContactStateKV<RD::ContactEnum::RF_FOOT, FD::StateEnum::I_P_RF>,
                                contactToContactStateKV<RD::ContactEnum::LH_FOOT, FD::StateEnum::I_P_LH>,
                                contactToContactStateKV<RD::ContactEnum::RH_FOOT, FD::StateEnum::I_P_RH>>;

  using mapContactToLandmarkUpdate =
      std_utils::CompileTimeMap<RD::ContactEnum, FD::ResidualEnum,
                                contactToResidualKV<RD::ContactEnum::LF_FOOT, FD::ResidualEnum::LMK_UPD_LF>,
                                contactToResidualKV<RD::ContactEnum::RF_FOOT, FD::ResidualEnum::LMK_UPD_RF>,
                                contactToResidualKV<RD::ContactEnum::LH_FOOT, FD::ResidualEnum::LMK_UPD_LH>,
                                contactToResidualKV<RD::ContactEnum::RH_FOOT, FD::ResidualEnum::LMK_UPD_RH>>;

  explicit ImuKinAnymalOdometryTsif(const ros::NodeHandle& node_handle) {
    const auto parameter_prefix = std::string{"ImuKinAnymalOdometryTsif"};
    //get filter parameters
    max_iter_ = param_io::param<int>(node_handle, parameter_prefix + std::string{"/filter/max_iter"}, 1);
    th_iter_ = param_io::param<double>(node_handle, parameter_prefix + std::string{"/filter/th_iter"}, 0.1);
    //get parameters for the residuals
    std::get<FD::ResidualEnum::POS_PRD>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/position_prediction"});
    std::get<FD::ResidualEnum::ATT_PRD>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/attitude_prediction"});
    std::get<FD::ResidualEnum::LVEL_PRD>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/linear_velocity_prediction"});
    std::get<FD::ResidualEnum::AVEL_UPD>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/angular_velocity_update"});
    std::get<FD::ResidualEnum::IBF_PRD>(residuals_)
        .LoadParameters(node_handle,
                        parameter_prefix + std::string{"/residuals/imu_linear_acceleration_bias_prediction"});
    std::get<FD::ResidualEnum::IBO_PRD>(residuals_)
        .LoadParameters(node_handle,
                        parameter_prefix + std::string{"/residuals/imu_angular_velocity_bias_prediction"});
    std::get<FD::ResidualEnum::LMK_UPD_RF>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/landmark_update"});
    std::get<FD::ResidualEnum::LMK_UPD_LF>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/landmark_update"});
    std::get<FD::ResidualEnum::LMK_UPD_LH>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/landmark_update"});
    std::get<FD::ResidualEnum::LMK_UPD_RH>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/landmark_update"});
    std::get<FD::ResidualEnum::LMK_PRD_RF>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/landmark_prediction"});
    std::get<FD::ResidualEnum::LMK_PRD_LF>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/landmark_prediction"});
    std::get<FD::ResidualEnum::LMK_PRD_LH>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/landmark_prediction"});
    std::get<FD::ResidualEnum::LMK_PRD_RH>(residuals_)
        .LoadParameters(node_handle, parameter_prefix + std::string{"/residuals/landmark_prediction"});
    //get parameters for the timelines
    SetMaxWaitTimes(
        param_io::param<double>(node_handle, parameter_prefix + std::string{"/timelines/max_wait_time"}, 0.1));
    //get parameters for the states
    I_init_.at(FD::StateEnum::I_R_IB) = param_io::param<double>(
        node_handle, parameter_prefix + std::string{"/states/I_r_IB/information"}, 1.);
    I_init_.at(FD::StateEnum::PHI_IB) = param_io::param<double>(
        node_handle, parameter_prefix + std::string{"/states/Phi_IB/information"}, 1.);
    I_init_.at(FD::StateEnum::B_V_IM) = param_io::param<double>(
        node_handle, parameter_prefix + std::string{"/states/B_v_IM/information"}, 1.);
    I_init_.at(FD::StateEnum::B_OMEGA_IB) = param_io::param<double>(
        node_handle, parameter_prefix + std::string{"/states/B_omega_IB/information"}, 1.);
    const double initialContactPointInformation = param_io::param<double>(
        node_handle, parameter_prefix + std::string{"/states/I_p_i/information"}, 1.);
    for (const auto contactKey : RD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      I_init_.at(mapContactToContactState::at(contactEnum)) =
          initialContactPointInformation;
    }
    I_init_.at(FD::StateEnum::B_B_F) = param_io::param<double>(
        node_handle, parameter_prefix + std::string{"/states/B_b_f/information"}, 1.);
    I_init_.at(FD::StateEnum::B_B_OMEGA) = param_io::param<double>(
        node_handle, parameter_prefix + std::string{"/states/B_b_omega/information"}, 1.);

    //set remaining parameters to defaults
    T_IB_init_ =
        kindr::HomTransformQuatD(kindr::Position3D(0.0, 0.0, 0.0), kindr::RotationQuaternionD(1., 0., 0., 0.));
    for (const auto contactKey : RD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      B_r_BSi_init_[contactEnum].setZero();
    }
    B_r_BM_ = Vec3::Zero();
    B_b_omega_init_ = Vec3::Zero();

    //initialize temporary gyro bias information
    I_B_b_omega_tmp_.setZero();
    I_B_f_omega_tmp_.setZero();
  }

  ~ImuKinAnymalOdometryTsif() override = default;

  //initialization function, called during the first uninitialized update of the TSIF
  void Init(TimePoint t) override {
    if (GetMinMaxTime() != TimePoint::min()) {
      startTime_ = t;
      time_ = t;
      //set initial state
      state_.Get<FD::StateEnum::I_R_IB>() = T_IB_init_.getPosition().toImplementation();
      state_.Get<FD::StateEnum::PHI_IB>() = T_IB_init_.getRotation().toImplementation();
      state_.Get<FD::StateEnum::PHI_IB>().normalize();
      state_.Get<FD::StateEnum::B_V_IM>().setZero();
      state_.Get<FD::StateEnum::B_OMEGA_IB>().setZero();
      const Mat3 C_IB = state_.Get<FD::StateEnum::PHI_IB>().toRotationMatrix();
      const Vec3 I_r_IB = state_.Get<FD::StateEnum::I_R_IB>();
      {
        const auto contactEnum = RD::ContactEnum::RF_FOOT;
        state_.Get<mapContactToContactState::at(contactEnum)>() =
            I_r_IB + C_IB * B_r_BSi_init_[contactEnum];
      }
      {
        const auto contactEnum = RD::ContactEnum::LF_FOOT;
        state_.Get<mapContactToContactState::at(contactEnum)>() =
            I_r_IB + C_IB * B_r_BSi_init_[contactEnum];
      }
      {
        const auto contactEnum = RD::ContactEnum::RH_FOOT;
        state_.Get<mapContactToContactState::at(contactEnum)>() =
            I_r_IB + C_IB * B_r_BSi_init_[contactEnum];
      }
      {
        const auto contactEnum = RD::ContactEnum::LH_FOOT;
        state_.Get<mapContactToContactState::at(contactEnum)>() =
            I_r_IB + C_IB * B_r_BSi_init_[contactEnum];
      }
      state_.Get<FD::StateEnum::B_B_F>().setZero();
      state_.Get<FD::StateEnum::B_B_OMEGA>() = B_b_omega_init_;
      //set unoptimized states (parameters)
      state_.Get<FD::ParamEnum::B_R_BM>() = B_r_BM_;
      //set initial information matrix
      I_.setIdentity();
      for (size_t i=0; i < FD::StateEnum::NUM_STATES; i++) {
        I_.block<3,3>(State::Start(i),State::Start(i)) = I_init_.at(i)*Mat3::Identity();
      }
      is_initialized_ = true;
    }
  }

  bool ToggleGyroBiasEstimation(bool toggle){

    bool success = false;
    constexpr int B_b_omega_index = State::Start(FD::StateEnum::B_B_OMEGA);

    if(is_initialized_){
      auto& enabled = std::get<FD::ResidualEnum::AVEL_UPD>(residuals_).bias_estimation_enabled_;
      if(enabled){
        if(!toggle){
          //store gyro bias covariance for later and zero out crossterms in covariance
          I_B_b_omega_tmp_ = I_.template block<3,3>(B_b_omega_index,B_b_omega_index);
          I_.template block<3,State::Dim()>(B_b_omega_index,0).setZero();
          I_.template block<State::Dim(),3>(0,B_b_omega_index).setZero();
          I_.template block<3,3>(B_b_omega_index,B_b_omega_index) = I_B_b_omega_tmp_;
          enabled = false;
          success = true;
        }
      }
      else {
        if(toggle){
          // resume estimation with correct covariance
          I_.template block<3,3>(B_b_omega_index,B_b_omega_index) = I_B_b_omega_tmp_;
          enabled = true;
          success = true;
        }
      }
    }

    return success;
  }

  bool ToggleAccelerometerBiasEstimation(bool toggle){

    bool success = false;
    constexpr int B_b_f_index = State::Start(FD::StateEnum::B_B_F);

    if(is_initialized_){
      auto& enabled = std::get<FD::ResidualEnum::LVEL_PRD>(residuals_).bias_estimation_enabled_;
      if(enabled){
        if(!toggle){
          //store gyro bias covariance for later and zero out crossterms in covariance
          I_B_f_omega_tmp_ = I_.template block<3,3>(B_b_f_index,B_b_f_index);
          I_.template block<3,State::Dim()>(B_b_f_index,0).setZero();
          I_.template block<State::Dim(),3>(0,B_b_f_index).setZero();
          I_.template block<3,3>(B_b_f_index,B_b_f_index) = I_B_f_omega_tmp_;
          enabled = false;
          success = true;
        }
      }
      else {
        if(toggle){
          // resume estimation with correct covariance
          I_.template block<3,3>(B_b_f_index,B_b_f_index) = I_B_f_omega_tmp_;
          enabled = true;
          success = true;
        }
      }
    }

    return success;
  }

  TimePoint GetLastImuStamp() const override { return std::get<FD::ResidualEnum::AVEL_UPD>(timelines_).GetLastTime(); }

  double GetLastLandmarkStamp() const override {
    return toSec(std::get<FD::ResidualEnum::LMK_UPD_RF>(timelines_).GetLastTime().time_since_epoch());
  }

  void AddMeasurementLinearAccelerationWorldToImuInBase(const TimePoint& time, const Vec3& B_f_IM) override {
    AddMeas<FD::ResidualEnum::LVEL_PRD>(time, std::make_shared<tsif::MeasAcc>(B_f_IM));
  }

  void AddMeasurementAngularVelocityWorldToBaseInBase(const TimePoint& time, const Vec3& B_omega_IB) override {
    AddMeas<FD::ResidualEnum::AVEL_UPD>(time, std::make_shared<tsif::MeasGyr>(B_omega_IB));
  }

  void AddMeasurementPositionsBaseToFootInBase(const TimePoint& time, const ContactEnumContainer<Vec3>& B_r_BSi) override {
    {
      constexpr auto contactEnum = RD::ContactEnum::RF_FOOT;
      AddMeas<mapContactToLandmarkUpdate::at(contactEnum)>(time,
                                                           std::make_shared<tsif::LandmarkInOdomMeasurement>(B_r_BSi[contactEnum]));
    }
    {
      constexpr auto contactEnum = RD::ContactEnum::LF_FOOT;
      AddMeas<mapContactToLandmarkUpdate::at(contactEnum)>(time,
                                                           std::make_shared<tsif::LandmarkInOdomMeasurement>(B_r_BSi[contactEnum]));
    }    {
      constexpr auto contactEnum = RD::ContactEnum::RH_FOOT;
      AddMeas<mapContactToLandmarkUpdate::at(contactEnum)>(time,
                                                           std::make_shared<tsif::LandmarkInOdomMeasurement>(B_r_BSi[contactEnum]));
    }
    {
      constexpr auto contactEnum = RD::ContactEnum::LH_FOOT;
      AddMeas<mapContactToLandmarkUpdate::at(contactEnum)>(time,
                                                           std::make_shared<tsif::LandmarkInOdomMeasurement>(B_r_BSi[contactEnum]));
    }
  }

  void AddMeasurementContacts(const ContactEnumContainer<anymal_state_estimator::Contact>& contacts, bool fakeKinematicsUpdateActive) override {
    {
      constexpr auto contactEnum = RD::ContactEnum::RF_FOOT;
      AddMeas<mapContactToLandmarkPrediction::at(contactEnum)>(TimePoint(tsif::fromSec(contacts[contactEnum].stamp_.toSeconds())),
                                                                   std::make_shared<tsif::BoolMeasurement>(contacts[contactEnum].flag_ || fakeKinematicsUpdateActive));
    }
    {
      constexpr auto contactEnum = RD::ContactEnum::LF_FOOT;
      AddMeas<mapContactToLandmarkPrediction::at(contactEnum)>(TimePoint(tsif::fromSec(contacts[contactEnum].stamp_.toSeconds())),
                                                                   std::make_shared<tsif::BoolMeasurement>(contacts[contactEnum].flag_ || fakeKinematicsUpdateActive));
    }
    {
      constexpr auto contactEnum = RD::ContactEnum::RH_FOOT;
      AddMeas<mapContactToLandmarkPrediction::at(contactEnum)>(TimePoint(tsif::fromSec(contacts[contactEnum].stamp_.toSeconds())),
                                                                   std::make_shared<tsif::BoolMeasurement>(contacts[contactEnum].flag_ || fakeKinematicsUpdateActive));
    }
    {
      constexpr auto contactEnum = RD::ContactEnum::LH_FOOT;
      AddMeas<mapContactToLandmarkPrediction::at(contactEnum)>(TimePoint(tsif::fromSec(contacts[contactEnum].stamp_.toSeconds())),
                                                                   std::make_shared<tsif::BoolMeasurement>(contacts[contactEnum].flag_ || fakeKinematicsUpdateActive));
    }
  }

  tsif::Vec3 GetPositionWorldToBaseInWorld() const override { return GetState().Get<FD::StateEnum::I_R_IB>(); }
  tsif::Quat GetOrientationBaseToWorld() const override { return GetState().Get<FD::StateEnum::PHI_IB>().normalized(); }
  tsif::Vec3 GetLinearVelocityWorldToImuInBase() const override { return GetState().Get<FD::StateEnum::B_V_IM>(); }
  tsif::Vec3 GetAngularVelocityWorldToBaseInBase() const override { return GetState().Get<FD::StateEnum::B_OMEGA_IB>(); }
  tsif::Vec3 GetLinearAccelerationBiasInBase() const override { return GetState().Get<FD::StateEnum::B_B_F>(); }
  tsif::Vec3 GetAngularVelocityBiasInBase() const override { return GetState().Get<FD::StateEnum::B_B_OMEGA>(); }

  int GetPositionWorldToBaseInWorldIndex() const override { return State::Start(FD::StateEnum::I_R_IB); }
  int GetOrientationBaseToWorldIndex() const override { return State::Start(FD::StateEnum::PHI_IB); }
  int GetLinearVelocityWorldToImuInBaseIndex() const override { return State::Start(FD::StateEnum::B_V_IM); }
  int GetAngularVelocityWorldToBaseInBaseIndex() const override { return State::Start(FD::StateEnum::B_OMEGA_IB); }
  int GetLinearAccelerationBiasInBaseIndex() const override { return State::Start(FD::StateEnum::B_B_F); }
  int GetAngularVelocityBiasInBaseIndex() const override { return State::Start(FD::StateEnum::B_B_OMEGA); }

 protected:
  //initial information diagonals
  std::array<double, FD::StateEnum::NUM_STATES> I_init_;

  //temporary storage for the imu bias information
  Mat3 I_B_b_omega_tmp_;
  Mat3 I_B_f_omega_tmp_;
};

} /* namespace tsif */
