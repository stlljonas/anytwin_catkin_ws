
/*!
 * @file    AnymalOdometryTsif.hpp
 * @author  Fabian Tresoldi
 * @date    July, 2018
 */

#pragma once

#include <tsif/utils/common.h>

#include <kindr/Core>

#include <romo/RobotModel.hpp>

#include <anymal_state_estimator/anymal_state_estimator_utils/Contact.hpp>

#include <tsif/filter_with_definition.h>

namespace tsif {

//odometry filter class for an Anymal adding some common functionality to the tsif::Filter,
//templated on a romo::ConcreteDescription and romo::RobotState
template <typename ConcreteRobotDescription_, typename RobotState_, typename FilterDefinition_>
class AnymalOdometryTsif : public FilterWithDefinition<FilterDefinition_> {
 public:

  using Base = FilterWithDefinition<FilterDefinition_>;
  using Base::FD;

  using RobotModel = typename romo::RobotModel<ConcreteRobotDescription_, RobotState_>;
  using RD = typename RobotModel::RD;
  using CT = typename ConcreteRobotDescription_::ConcreteTopology;
  using ContactEnum = typename CT::ContactEnum;

  template <typename T>
  using ContactEnumContainer = std_utils::EnumArray<ContactEnum, T>;

  AnymalOdometryTsif() = default;
  ~AnymalOdometryTsif() override = default;

  void SetInitPose(const kindr::HomTransformQuatD& T_IB_init) { T_IB_init_ = T_IB_init; }
  void SetImuOffset(const Vec3& B_r_BM) { B_r_BM_ = B_r_BM; }
  void SetInitContacts(const ContactEnumContainer<Vec3>& B_r_BSi_init) { B_r_BSi_init_ = B_r_BSi_init; }
  kindr::HomTransformQuatD& GetInitPose() { return T_IB_init_; }
  Vec3& GetInitGyroBias() { return B_b_omega_init_; }
  const kindr::HomTransformQuatD& GetInitPose() const { return T_IB_init_; }

  virtual bool ToggleGyroBiasEstimation(bool toggle) = 0;
  virtual bool ToggleAccelerometerBiasEstimation(bool toggle) = 0;

  virtual TimePoint GetLastImuStamp() const = 0;
  virtual double GetLastLandmarkStamp() const = 0;

  virtual void AddMeasurementLinearAccelerationWorldToImuInBase(const TimePoint& time, const Vec3& B_f_IM) = 0;
  virtual void AddMeasurementAngularVelocityWorldToBaseInBase(const TimePoint& time, const Vec3& B_omega_IB) = 0;
  virtual void AddMeasurementPositionsBaseToFootInBase(const TimePoint& time, const ContactEnumContainer<Vec3>& B_r_BSi) = 0;
  virtual void AddMeasurementContacts(const ContactEnumContainer<anymal_state_estimator::Contact>& contacts, bool fakeKinematicsUpdateActive) = 0;

  virtual tsif::Vec3 GetPositionWorldToBaseInWorld() const = 0;
  virtual tsif::Quat GetOrientationBaseToWorld() const = 0;
  virtual tsif::Vec3 GetLinearVelocityWorldToImuInBase() const = 0;
  virtual tsif::Vec3 GetAngularVelocityWorldToBaseInBase() const = 0;
  virtual tsif::Vec3 GetLinearAccelerationBiasInBase() const = 0;
  virtual tsif::Vec3 GetAngularVelocityBiasInBase() const = 0;

  virtual int GetPositionWorldToBaseInWorldIndex() const = 0;
  virtual int GetOrientationBaseToWorldIndex() const = 0;
  virtual int GetLinearVelocityWorldToImuInBaseIndex() const = 0;
  virtual int GetAngularVelocityWorldToBaseInBaseIndex() const = 0;
  virtual int GetLinearAccelerationBiasInBaseIndex() const = 0;
  virtual int GetAngularVelocityBiasInBaseIndex() const = 0;

 protected:
  //initial pose base to odom
  kindr::HomTransformQuatD T_IB_init_;
  //initial gyro bias
  Vec3 B_b_omega_init_;
  //initial contact points, i.e. array of positions base to contact in base
  ContactEnumContainer<Vec3> B_r_BSi_init_;
  //parameter for the position base-to-imu in base frame
  Vec3 B_r_BM_;
};

//check if some type is an implementation of AnymalOdometryTsif and compatible with a given Description/State
template <typename ConcreteRobotDescription_, typename RobotState_, typename Derived_>
struct is_anymal_odometry_tsif_compatible
{
    using D = typename std::remove_cv<Derived_>::type;
    template <typename T>
    static std::true_type test(AnymalOdometryTsif<ConcreteRobotDescription_, RobotState_, T>*);
    static std::false_type test(void*);
    using type = decltype(test(std::declval<D*>()));
};
template <typename ConcreteRobotDescription_, typename RobotState_, typename Derived_>
using is_anymal_odometry_tsif_compatible_t = typename is_anymal_odometry_tsif_compatible<ConcreteRobotDescription_, RobotState_, Derived_>::type;

} /* namespace tsif */
