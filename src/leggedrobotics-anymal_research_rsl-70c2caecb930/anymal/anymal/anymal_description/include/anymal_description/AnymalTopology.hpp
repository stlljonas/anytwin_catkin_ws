/*!
 * @file     AnymalTopology.hpp
 * @author   Dario Bellicoso
 * @date     Nov 6, 2017
 */

#pragma once

// std_utils
#include <std_utils/std_utils.hpp>

namespace anymal_description {

struct AnymalTopology {
// Declare Enums
#define ANYMAL_LIMB_NAMES LF_LEG, RF_LEG, LH_LEG, RH_LEG
  CONSECUTIVE_ENUM_FROM_LIST(LimbEnum, ANYMAL_LIMB_NAMES)
  using ct_limbKeys = std_utils::CompileTimeKeys<LimbEnum, std_utils::CompileTimeKey<LimbEnum, LimbEnum::LF_LEG, ct_string("LF_LEG")>,
                                                 std_utils::CompileTimeKey<LimbEnum, LimbEnum::RF_LEG, ct_string("RF_LEG")>,
                                                 std_utils::CompileTimeKey<LimbEnum, LimbEnum::LH_LEG, ct_string("LH_LEG")>,
                                                 std_utils::CompileTimeKey<LimbEnum, LimbEnum::RH_LEG, ct_string("RH_LEG")>>;
  static constexpr auto limbKeys = ct_limbKeys::getKeysArray();

#define ANYMAL_BRANCH_NAMES BASE, ANYMAL_LIMB_NAMES
  CONSECUTIVE_ENUM_FROM_LIST(BranchEnum, ANYMAL_BRANCH_NAMES)
  using ct_branchKeys = std_utils::CompileTimeKeys<BranchEnum, std_utils::CompileTimeKey<BranchEnum, BranchEnum::BASE, ct_string("BASE")>,
                                                   std_utils::CompileTimeKey<BranchEnum, BranchEnum::LF_LEG, ct_string("LF_LEG")>,
                                                   std_utils::CompileTimeKey<BranchEnum, BranchEnum::RF_LEG, ct_string("RF_LEG")>,
                                                   std_utils::CompileTimeKey<BranchEnum, BranchEnum::LH_LEG, ct_string("LH_LEG")>,
                                                   std_utils::CompileTimeKey<BranchEnum, BranchEnum::RH_LEG, ct_string("RH_LEG")>>;
  static constexpr auto branchKeys = ct_branchKeys::getKeysArray();

#define ANYMAL_LATERAL_NAMES LEFT, RIGHT
  CONSECUTIVE_ENUM_FROM_LIST(LateralEnum, ANYMAL_LATERAL_NAMES)
  using ct_lateralKeys =
      std_utils::CompileTimeKeys<LateralEnum, std_utils::CompileTimeKey<LateralEnum, LateralEnum::LEFT, ct_string("LEFT")>,
                                 std_utils::CompileTimeKey<LateralEnum, LateralEnum::RIGHT, ct_string("RIGHT")>>;
  static constexpr auto lateralKeys = ct_lateralKeys::getKeysArray();

#define ANYMAL_LONGITUDINAL_NAMES FORE, HIND
  CONSECUTIVE_ENUM_FROM_LIST(LongitudinalEnum, ANYMAL_LONGITUDINAL_NAMES)
  using ct_longitudinalKeys =
      std_utils::CompileTimeKeys<LongitudinalEnum, std_utils::CompileTimeKey<LongitudinalEnum, LongitudinalEnum::FORE, ct_string("FORE")>,
                                 std_utils::CompileTimeKey<LongitudinalEnum, LongitudinalEnum::HIND, ct_string("HIND")>>;
  static constexpr auto longitudinalKeys = ct_longitudinalKeys::getKeysArray();

#define ANYMAL_LEG_CONFIGURATION_NAMES OConfiguration, XConfiguration
  CONSECUTIVE_ENUM_FROM_LIST(LegConfigEnum, ANYMAL_LEG_CONFIGURATION_NAMES)
  using ct_legConfigKeys =
      std_utils::CompileTimeKeys<LegConfigEnum,
                                 std_utils::CompileTimeKey<LegConfigEnum, LegConfigEnum::OConfiguration, ct_string("OConfiguration")>,
                                 std_utils::CompileTimeKey<LegConfigEnum, LegConfigEnum::XConfiguration, ct_string("XConfiguration")>>;
  static constexpr auto legConfigKeys = ct_legConfigKeys::getKeysArray();

#define ANYMAL_JOINT_NODE_NAMES HAA, HFE, KFE
  CONSECUTIVE_ENUM_FROM_LIST(JointNodeEnum, ANYMAL_JOINT_NODE_NAMES)
  using ct_jointNodeKeys =
      std_utils::CompileTimeKeys<JointNodeEnum, std_utils::CompileTimeKey<JointNodeEnum, JointNodeEnum::HAA, ct_string("HAA")>,
                                 std_utils::CompileTimeKey<JointNodeEnum, JointNodeEnum::HFE, ct_string("HFE")>,
                                 std_utils::CompileTimeKey<JointNodeEnum, JointNodeEnum::KFE, ct_string("KFE")>>;
  static constexpr auto jointNodeKeys = ct_jointNodeKeys::getKeysArray();

#define ANYMAL_JOINT_NAMES LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE
  CONSECUTIVE_ENUM_FROM_LIST(JointEnum, ANYMAL_JOINT_NAMES)
  using ct_jointKeys = std_utils::CompileTimeKeys<JointEnum, std_utils::CompileTimeKey<JointEnum, JointEnum::LF_HAA, ct_string("LF_HAA")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::LF_HFE, ct_string("LF_HFE")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::LF_KFE, ct_string("LF_KFE")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::RF_HAA, ct_string("RF_HAA")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::RF_HFE, ct_string("RF_HFE")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::RF_KFE, ct_string("RF_KFE")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::LH_HAA, ct_string("LH_HAA")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::LH_HFE, ct_string("LH_HFE")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::LH_KFE, ct_string("LH_KFE")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::RH_HAA, ct_string("RH_HAA")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::RH_HFE, ct_string("RH_HFE")>,
                                                  std_utils::CompileTimeKey<JointEnum, JointEnum::RH_KFE, ct_string("RH_KFE")>>;
  static constexpr auto jointKeys = ct_jointKeys::getKeysArray();

#define ANYMAL_ACTUATOR_NAMES ANYMAL_JOINT_NAMES
  CONSECUTIVE_ENUM_FROM_LIST(ActuatorEnum, ANYMAL_ACTUATOR_NAMES)
  using ct_actuatorKeys =
      std_utils::CompileTimeKeys<ActuatorEnum, std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::LF_HAA, ct_string("LF_HAA")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::LF_HFE, ct_string("LF_HFE")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::LF_KFE, ct_string("LF_KFE")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::RF_HAA, ct_string("RF_HAA")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::RF_HFE, ct_string("RF_HFE")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::RF_KFE, ct_string("RF_KFE")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::LH_HAA, ct_string("LH_HAA")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::LH_HFE, ct_string("LH_HFE")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::LH_KFE, ct_string("LH_KFE")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::RH_HAA, ct_string("RH_HAA")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::RH_HFE, ct_string("RH_HFE")>,
                                 std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::RH_KFE, ct_string("RH_KFE")>>;
  static constexpr auto actuatorKeys = ct_actuatorKeys::getKeysArray();

#define ANYMAL_ACTUATOR_NODE_NAMES HAA, HFE, KFE
  CONSECUTIVE_ENUM_FROM_LIST(ActuatorNodeEnum, ANYMAL_ACTUATOR_NODE_NAMES)
  using ct_actuatorNodeKeys =
      std_utils::CompileTimeKeys<ActuatorNodeEnum, std_utils::CompileTimeKey<ActuatorNodeEnum, ActuatorNodeEnum::HAA, ct_string("HAA")>,
                                 std_utils::CompileTimeKey<ActuatorNodeEnum, ActuatorNodeEnum::HFE, ct_string("HFE")>,
                                 std_utils::CompileTimeKey<ActuatorNodeEnum, ActuatorNodeEnum::KFE, ct_string("KFE")>>;
  static constexpr auto actuatorNodeKeys = ct_actuatorNodeKeys::getKeysArray();

#define ANYMAL_BODY_NODE_NAMES BASE, HIP, THIGH, SHANK, FOOT
  CONSECUTIVE_ENUM_FROM_LIST(BodyNodeEnum, ANYMAL_BODY_NODE_NAMES)
  using ct_bodyNodeKeys =
      std_utils::CompileTimeKeys<BodyNodeEnum, std_utils::CompileTimeKey<BodyNodeEnum, BodyNodeEnum::BASE, ct_string("BASE")>,
                                 std_utils::CompileTimeKey<BodyNodeEnum, BodyNodeEnum::HIP, ct_string("HIP")>,
                                 std_utils::CompileTimeKey<BodyNodeEnum, BodyNodeEnum::THIGH, ct_string("THIGH")>,
                                 std_utils::CompileTimeKey<BodyNodeEnum, BodyNodeEnum::SHANK, ct_string("SHANK")>,
                                 std_utils::CompileTimeKey<BodyNodeEnum, BodyNodeEnum::FOOT, ct_string("FOOT")>>;
  static constexpr auto bodyNodeKeys = ct_bodyNodeKeys::getKeysArray();

#define ANYMAL_BODY_NAMES                                                                                                                \
  BASE, LF_HIP, LF_THIGH, LF_SHANK, LF_FOOT, RF_HIP, RF_THIGH, RF_SHANK, RF_FOOT, LH_HIP, LH_THIGH, LH_SHANK, LH_FOOT, RH_HIP, RH_THIGH, \
      RH_SHANK, RH_FOOT
  CONSECUTIVE_ENUM_FROM_LIST(BodyEnum, ANYMAL_BODY_NAMES)
  using ct_bodyKeys = std_utils::CompileTimeKeys<BodyEnum, std_utils::CompileTimeKey<BodyEnum, BodyEnum::BASE, ct_string("base")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::LF_HIP, ct_string("LF_HIP")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::LF_THIGH, ct_string("LF_THIGH")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::LF_SHANK, ct_string("LF_SHANK")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::LF_FOOT, ct_string("LF_FOOT")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::RF_HIP, ct_string("RF_HIP")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::RF_THIGH, ct_string("RF_THIGH")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::RF_SHANK, ct_string("RF_SHANK")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::RF_FOOT, ct_string("RF_FOOT")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::LH_HIP, ct_string("LH_HIP")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::LH_THIGH, ct_string("LH_THIGH")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::LH_SHANK, ct_string("LH_SHANK")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::LH_FOOT, ct_string("LH_FOOT")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::RH_HIP, ct_string("RH_HIP")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::RH_THIGH, ct_string("RH_THIGH")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::RH_SHANK, ct_string("RH_SHANK")>,
                                                 std_utils::CompileTimeKey<BodyEnum, BodyEnum::RH_FOOT, ct_string("RH_FOOT")>>;
  static constexpr auto bodyKeys = ct_bodyKeys::getKeysArray();

#define ANYMAL_GENERALIZED_COORDINATE_NAMES X, Y, Z, Q_W, Q_X, Q_Y, Q_Z, ANYMAL_JOINT_NAMES
  CONSECUTIVE_ENUM_FROM_LIST(GeneralizedCoordinatesEnum, ANYMAL_GENERALIZED_COORDINATE_NAMES)
  using ct_generalizedCoordinateKeys = std_utils::CompileTimeKeys<
      GeneralizedCoordinatesEnum, std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::X, ct_string("X")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Y, ct_string("Y")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Z, ct_string("Z")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Q_W, ct_string("Q_W")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Q_X, ct_string("Q_X")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Q_Y, ct_string("Q_Y")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Q_Z, ct_string("Q_Z")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::LF_HAA, ct_string("LF_HAA")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::LF_HFE, ct_string("LF_HFE")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::LF_KFE, ct_string("LF_KFE")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::RF_HAA, ct_string("RF_HAA")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::RF_HFE, ct_string("RF_HFE")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::RF_KFE, ct_string("RF_KFE")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::LH_HAA, ct_string("LH_HAA")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::LH_HFE, ct_string("LH_HFE")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::LH_KFE, ct_string("LH_KFE")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::RH_HAA, ct_string("RH_HAA")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::RH_HFE, ct_string("RH_HFE")>,
      std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::RH_KFE, ct_string("RH_KFE")>>;
  static constexpr auto generalizedCoordinateKeys = ct_generalizedCoordinateKeys::getKeysArray();

#define ANYMAL_GENERALIZED_VELOCITY_NAMES L_X, L_Y, L_Z, A_X, A_Y, A_Z, ANYMAL_JOINT_NAMES
  CONSECUTIVE_ENUM_FROM_LIST(GeneralizedVelocitiesEnum, ANYMAL_GENERALIZED_VELOCITY_NAMES)
  using ct_generalizedVelocityKeys = std_utils::CompileTimeKeys<
      GeneralizedVelocitiesEnum, std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::L_X, ct_string("L_X")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::L_Y, ct_string("L_Y")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::L_Z, ct_string("L_Z")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::A_X, ct_string("A_X")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::A_Y, ct_string("A_Y")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::A_Z, ct_string("A_Z")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::LF_HAA, ct_string("LF_HAA")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::LF_HFE, ct_string("LF_HFE")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::LF_KFE, ct_string("LF_KFE")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::RF_HAA, ct_string("RF_HAA")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::RF_HFE, ct_string("RF_HFE")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::RF_KFE, ct_string("RF_KFE")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::LH_HAA, ct_string("LH_HAA")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::LH_HFE, ct_string("LH_HFE")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::LH_KFE, ct_string("LH_KFE")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::RH_HAA, ct_string("RH_HAA")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::RH_HFE, ct_string("RH_HFE")>,
      std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::RH_KFE, ct_string("RH_KFE")>>;
  static constexpr auto generalizedVelocityKeys = ct_generalizedVelocityKeys::getKeysArray();

#define ANYMAL_FOOT_NAMES LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT
  CONSECUTIVE_ENUM_FROM_LIST(FootEnum, ANYMAL_FOOT_NAMES)
  using ct_footKeys = std_utils::CompileTimeKeys<FootEnum, std_utils::CompileTimeKey<FootEnum, FootEnum::LF_FOOT, ct_string("LF_FOOT")>,
                                                 std_utils::CompileTimeKey<FootEnum, FootEnum::RF_FOOT, ct_string("RF_FOOT")>,
                                                 std_utils::CompileTimeKey<FootEnum, FootEnum::LH_FOOT, ct_string("LH_FOOT")>,
                                                 std_utils::CompileTimeKey<FootEnum, FootEnum::RH_FOOT, ct_string("RH_FOOT")>>;
  static constexpr auto footKeys = ct_footKeys::getKeysArray();

#define ANYMAL_CONTACT_NAMES ANYMAL_FOOT_NAMES
  CONSECUTIVE_ENUM_FROM_LIST(ContactEnum, ANYMAL_CONTACT_NAMES)
  using ct_contactKeys =
      std_utils::CompileTimeKeys<ContactEnum, std_utils::CompileTimeKey<ContactEnum, ContactEnum::LF_FOOT, ct_string("LF_FOOT")>,
                                 std_utils::CompileTimeKey<ContactEnum, ContactEnum::RF_FOOT, ct_string("RF_FOOT")>,
                                 std_utils::CompileTimeKey<ContactEnum, ContactEnum::LH_FOOT, ct_string("LH_FOOT")>,
                                 std_utils::CompileTimeKey<ContactEnum, ContactEnum::RH_FOOT, ct_string("RH_FOOT")>>;
  static constexpr auto contactKeys = ct_contactKeys::getKeysArray();

#define ANYMAL_CONTACT_STATE_NAMES OPEN, CLOSED, SLIPPING
  CONSECUTIVE_ENUM_FROM_LIST(ContactStateEnum, ANYMAL_CONTACT_STATE_NAMES)
  using ct_contactStateKeys =
      std_utils::CompileTimeKeys<ContactStateEnum, std_utils::CompileTimeKey<ContactStateEnum, ContactStateEnum::OPEN, ct_string("OPEN")>,
                                 std_utils::CompileTimeKey<ContactStateEnum, ContactStateEnum::CLOSED, ct_string("CLOSED")>,
                                 std_utils::CompileTimeKey<ContactStateEnum, ContactStateEnum::SLIPPING, ct_string("SLIPPING")>>;
  static constexpr auto contactStateKeys = ct_contactStateKeys::getKeysArray();

#define ANYMAL_COORDINATE_FRAMES_NAMES WORLD, BASE, BODY
  CONSECUTIVE_ENUM_FROM_LIST(CoordinateFrameEnum, ANYMAL_COORDINATE_FRAMES_NAMES)
  using ct_coordinateFrameKeys =
      std_utils::CompileTimeKeys<CoordinateFrameEnum,
                                 std_utils::CompileTimeKey<CoordinateFrameEnum, CoordinateFrameEnum::WORLD, ct_string("world")>,
                                 std_utils::CompileTimeKey<CoordinateFrameEnum, CoordinateFrameEnum::BASE, ct_string("base")>,
                                 std_utils::CompileTimeKey<CoordinateFrameEnum, CoordinateFrameEnum::BODY, ct_string("body")>>;
  static constexpr auto coordinateFrameKeys = ct_coordinateFrameKeys::getKeysArray();

#define ANYMAL_FRAME_TRANSFORM_NAMES FootprintToOdom, FeetcenterToOdom
  CONSECUTIVE_ENUM_FROM_LIST(FrameTransformEnum, ANYMAL_FRAME_TRANSFORM_NAMES)
  using ct_frameTransformKeys = std_utils::CompileTimeKeys<
      FrameTransformEnum,
      std_utils::CompileTimeKey<FrameTransformEnum, FrameTransformEnum::FootprintToOdom, ct_string("footprint_to_odom")>,
      std_utils::CompileTimeKey<FrameTransformEnum, FrameTransformEnum::FeetcenterToOdom, ct_string("feetcenter_to_odom")>>;
  static constexpr auto frameTransformKeys = ct_frameTransformKeys::getKeysArray();

  template <ActuatorEnum actuator, LimbEnum limb>
  using actuatorToLimbKV = std_utils::KeyValuePair<ActuatorEnum, LimbEnum, actuator, limb>;
  using mapActuatorEnumToLimbEnum = std_utils::CompileTimeMap<
      ActuatorEnum, LimbEnum, actuatorToLimbKV<ActuatorEnum::LF_HAA, LimbEnum::LF_LEG>,
      actuatorToLimbKV<ActuatorEnum::LF_HFE, LimbEnum::LF_LEG>, actuatorToLimbKV<ActuatorEnum::LF_KFE, LimbEnum::LF_LEG>,
      actuatorToLimbKV<ActuatorEnum::RF_HAA, LimbEnum::RF_LEG>, actuatorToLimbKV<ActuatorEnum::RF_HFE, LimbEnum::RF_LEG>,
      actuatorToLimbKV<ActuatorEnum::RF_KFE, LimbEnum::RF_LEG>, actuatorToLimbKV<ActuatorEnum::LH_HAA, LimbEnum::LH_LEG>,
      actuatorToLimbKV<ActuatorEnum::LH_HFE, LimbEnum::LH_LEG>, actuatorToLimbKV<ActuatorEnum::LH_KFE, LimbEnum::LH_LEG>,
      actuatorToLimbKV<ActuatorEnum::RH_HAA, LimbEnum::RH_LEG>, actuatorToLimbKV<ActuatorEnum::RH_HFE, LimbEnum::RH_LEG>,
      actuatorToLimbKV<ActuatorEnum::RH_KFE, LimbEnum::RH_LEG>>;

  template <JointEnum joint, JointNodeEnum jointNode>
  using jointToJointNodeKV = std_utils::KeyValuePair<JointEnum, JointNodeEnum, joint, jointNode>;
  using mapJointEnumToJointNodeEnum = std_utils::CompileTimeMap<
      JointEnum, JointNodeEnum, jointToJointNodeKV<JointEnum::LF_HAA, JointNodeEnum::HAA>,
      jointToJointNodeKV<JointEnum::LF_HFE, JointNodeEnum::HFE>, jointToJointNodeKV<JointEnum::LF_KFE, JointNodeEnum::KFE>,
      jointToJointNodeKV<JointEnum::RF_HFE, JointNodeEnum::HFE>, jointToJointNodeKV<JointEnum::RF_HAA, JointNodeEnum::HAA>,
      jointToJointNodeKV<JointEnum::RF_KFE, JointNodeEnum::KFE>, jointToJointNodeKV<JointEnum::LH_HAA, JointNodeEnum::HAA>,
      jointToJointNodeKV<JointEnum::LH_HFE, JointNodeEnum::HFE>, jointToJointNodeKV<JointEnum::LH_KFE, JointNodeEnum::KFE>,
      jointToJointNodeKV<JointEnum::RH_HAA, JointNodeEnum::HAA>, jointToJointNodeKV<JointEnum::RH_HFE, JointNodeEnum::HFE>,
      jointToJointNodeKV<JointEnum::RH_KFE, JointNodeEnum::KFE>>;

  template <ActuatorEnum actuator, ActuatorNodeEnum actuatorNode>
  using actuatorToActuatorNodeKV = std_utils::KeyValuePair<ActuatorEnum, ActuatorNodeEnum, actuator, actuatorNode>;
  using mapActuatorEnumToActuatorNodeEnum =
      std_utils::CompileTimeMap<ActuatorEnum, ActuatorNodeEnum, actuatorToActuatorNodeKV<ActuatorEnum::LF_HAA, ActuatorNodeEnum::HAA>,
                                actuatorToActuatorNodeKV<ActuatorEnum::LF_HFE, ActuatorNodeEnum::HFE>,
                                actuatorToActuatorNodeKV<ActuatorEnum::LF_KFE, ActuatorNodeEnum::KFE>,
                                actuatorToActuatorNodeKV<ActuatorEnum::RF_HFE, ActuatorNodeEnum::HFE>,
                                actuatorToActuatorNodeKV<ActuatorEnum::RF_HAA, ActuatorNodeEnum::HAA>,
                                actuatorToActuatorNodeKV<ActuatorEnum::RF_KFE, ActuatorNodeEnum::KFE>,
                                actuatorToActuatorNodeKV<ActuatorEnum::LH_HAA, ActuatorNodeEnum::HAA>,
                                actuatorToActuatorNodeKV<ActuatorEnum::LH_HFE, ActuatorNodeEnum::HFE>,
                                actuatorToActuatorNodeKV<ActuatorEnum::LH_KFE, ActuatorNodeEnum::KFE>,
                                actuatorToActuatorNodeKV<ActuatorEnum::RH_HAA, ActuatorNodeEnum::HAA>,
                                actuatorToActuatorNodeKV<ActuatorEnum::RH_HFE, ActuatorNodeEnum::HFE>,
                                actuatorToActuatorNodeKV<ActuatorEnum::RH_KFE, ActuatorNodeEnum::KFE>>;

  template <ActuatorEnum actuator, JointEnum joint>
  using actuatorToJointKV = std_utils::KeyValuePair<ActuatorEnum, JointEnum, actuator, joint>;
  using mapActuatorEnumToJointEnum = std_utils::CompileTimeMap<
      ActuatorEnum, JointEnum, actuatorToJointKV<ActuatorEnum::LF_HAA, JointEnum::LF_HAA>,
      actuatorToJointKV<ActuatorEnum::LF_HFE, JointEnum::LF_HFE>, actuatorToJointKV<ActuatorEnum::LF_KFE, JointEnum::LF_KFE>,
      actuatorToJointKV<ActuatorEnum::RF_HAA, JointEnum::RF_HAA>, actuatorToJointKV<ActuatorEnum::RF_HFE, JointEnum::RF_HFE>,
      actuatorToJointKV<ActuatorEnum::RF_KFE, JointEnum::RF_KFE>, actuatorToJointKV<ActuatorEnum::LH_HAA, JointEnum::LH_HAA>,
      actuatorToJointKV<ActuatorEnum::LH_HFE, JointEnum::LH_HFE>, actuatorToJointKV<ActuatorEnum::LH_KFE, JointEnum::LH_KFE>,
      actuatorToJointKV<ActuatorEnum::RH_HAA, JointEnum::RH_HAA>, actuatorToJointKV<ActuatorEnum::RH_HFE, JointEnum::RH_HFE>,
      actuatorToJointKV<ActuatorEnum::RH_KFE, JointEnum::RH_KFE>>;

  template <JointEnum joint, ActuatorEnum actuator>
  using jointToActuatorKV = std_utils::KeyValuePair<JointEnum, ActuatorEnum, joint, actuator>;
  using mapJointEnumToActuatorEnum = std_utils::CompileTimeMap<
      JointEnum, ActuatorEnum, jointToActuatorKV<JointEnum::LF_HAA, ActuatorEnum::LF_HAA>,
      jointToActuatorKV<JointEnum::LF_HFE, ActuatorEnum::LF_HFE>, jointToActuatorKV<JointEnum::LF_KFE, ActuatorEnum::LF_KFE>,
      jointToActuatorKV<JointEnum::RF_HAA, ActuatorEnum::RF_HAA>, jointToActuatorKV<JointEnum::RF_HFE, ActuatorEnum::RF_HFE>,
      jointToActuatorKV<JointEnum::RF_KFE, ActuatorEnum::RF_KFE>, jointToActuatorKV<JointEnum::LH_HAA, ActuatorEnum::LH_HAA>,
      jointToActuatorKV<JointEnum::LH_HFE, ActuatorEnum::LH_HFE>, jointToActuatorKV<JointEnum::LH_KFE, ActuatorEnum::LH_KFE>,
      jointToActuatorKV<JointEnum::RH_HAA, ActuatorEnum::RH_HAA>, jointToActuatorKV<JointEnum::RH_HFE, ActuatorEnum::RH_HFE>,
      jointToActuatorKV<JointEnum::RH_KFE, ActuatorEnum::RH_KFE>>;

  template <BodyEnum body, BranchEnum branch>
  using bodyToBranchKV = std_utils::KeyValuePair<BodyEnum, BranchEnum, body, branch>;
  using mapBodyEnumToBranchEnum = std_utils::CompileTimeMap<
      BodyEnum, BranchEnum, bodyToBranchKV<BodyEnum::BASE, BranchEnum::BASE>, bodyToBranchKV<BodyEnum::LF_HIP, BranchEnum::LF_LEG>,
      bodyToBranchKV<BodyEnum::LF_THIGH, BranchEnum::LF_LEG>, bodyToBranchKV<BodyEnum::LF_SHANK, BranchEnum::LF_LEG>,
      bodyToBranchKV<BodyEnum::LF_FOOT, BranchEnum::LF_LEG>, bodyToBranchKV<BodyEnum::RF_HIP, BranchEnum::RF_LEG>,
      bodyToBranchKV<BodyEnum::RF_THIGH, BranchEnum::RF_LEG>, bodyToBranchKV<BodyEnum::RF_SHANK, BranchEnum::RF_LEG>,
      bodyToBranchKV<BodyEnum::RF_FOOT, BranchEnum::RF_LEG>, bodyToBranchKV<BodyEnum::LH_HIP, BranchEnum::LH_LEG>,
      bodyToBranchKV<BodyEnum::LH_THIGH, BranchEnum::LH_LEG>, bodyToBranchKV<BodyEnum::LH_SHANK, BranchEnum::LH_LEG>,
      bodyToBranchKV<BodyEnum::LH_FOOT, BranchEnum::LH_LEG>, bodyToBranchKV<BodyEnum::RH_HIP, BranchEnum::RH_LEG>,
      bodyToBranchKV<BodyEnum::RH_THIGH, BranchEnum::RH_LEG>, bodyToBranchKV<BodyEnum::RH_SHANK, BranchEnum::RH_LEG>,
      bodyToBranchKV<BodyEnum::RH_FOOT, BranchEnum::RH_LEG>>;

  template <BodyEnum body, BodyNodeEnum bodyNode>
  using bodyToBodyNodeKV = std_utils::KeyValuePair<BodyEnum, BodyNodeEnum, body, bodyNode>;
  using mapBodyEnumToBodyNodeEnum = std_utils::CompileTimeMap<
      BodyEnum, BodyNodeEnum, bodyToBodyNodeKV<BodyEnum::BASE, BodyNodeEnum::BASE>, bodyToBodyNodeKV<BodyEnum::LF_HIP, BodyNodeEnum::HIP>,
      bodyToBodyNodeKV<BodyEnum::LF_THIGH, BodyNodeEnum::THIGH>, bodyToBodyNodeKV<BodyEnum::LF_SHANK, BodyNodeEnum::SHANK>,
      bodyToBodyNodeKV<BodyEnum::LF_FOOT, BodyNodeEnum::FOOT>, bodyToBodyNodeKV<BodyEnum::RF_HIP, BodyNodeEnum::HIP>,
      bodyToBodyNodeKV<BodyEnum::RF_THIGH, BodyNodeEnum::THIGH>, bodyToBodyNodeKV<BodyEnum::RF_SHANK, BodyNodeEnum::SHANK>,
      bodyToBodyNodeKV<BodyEnum::RF_FOOT, BodyNodeEnum::FOOT>, bodyToBodyNodeKV<BodyEnum::LH_HIP, BodyNodeEnum::HIP>,
      bodyToBodyNodeKV<BodyEnum::LH_THIGH, BodyNodeEnum::THIGH>, bodyToBodyNodeKV<BodyEnum::LH_SHANK, BodyNodeEnum::SHANK>,
      bodyToBodyNodeKV<BodyEnum::LH_FOOT, BodyNodeEnum::FOOT>, bodyToBodyNodeKV<BodyEnum::RH_HIP, BodyNodeEnum::HIP>,
      bodyToBodyNodeKV<BodyEnum::RH_THIGH, BodyNodeEnum::THIGH>, bodyToBodyNodeKV<BodyEnum::RH_SHANK, BodyNodeEnum::SHANK>,
      bodyToBodyNodeKV<BodyEnum::RH_FOOT, BodyNodeEnum::FOOT>>;

  template <BranchEnum branch, LimbEnum limb>
  using branchToLimbKV = std_utils::KeyValuePair<BranchEnum, LimbEnum, branch, limb>;
  using mapBranchEnumToLimbEnum =
      std_utils::CompileTimeMap<BranchEnum, LimbEnum, branchToLimbKV<BranchEnum::LF_LEG, LimbEnum::LF_LEG>,
                                branchToLimbKV<BranchEnum::RF_LEG, LimbEnum::RF_LEG>, branchToLimbKV<BranchEnum::LH_LEG, LimbEnum::LH_LEG>,
                                branchToLimbKV<BranchEnum::RH_LEG, LimbEnum::RH_LEG>>;

  template <ContactEnum contact, BodyEnum body>
  using contactToBodyKV = std_utils::KeyValuePair<ContactEnum, BodyEnum, contact, body>;
  using mapContactEnumToBodyEnum =
      std_utils::CompileTimeMap<ContactEnum, BodyEnum, contactToBodyKV<ContactEnum::LF_FOOT, BodyEnum::LF_FOOT>,
                                contactToBodyKV<ContactEnum::RF_FOOT, BodyEnum::RF_FOOT>,
                                contactToBodyKV<ContactEnum::LH_FOOT, BodyEnum::LH_FOOT>,
                                contactToBodyKV<ContactEnum::RH_FOOT, BodyEnum::RH_FOOT>>;

  template <FootEnum foot, BranchEnum branch>
  using footToBranchKV = std_utils::KeyValuePair<FootEnum, BranchEnum, foot, branch>;
  using mapFootEnumToBranchEnum = std_utils::CompileTimeMap<
      FootEnum, BranchEnum, footToBranchKV<FootEnum::LF_FOOT, BranchEnum::LF_LEG>, footToBranchKV<FootEnum::RF_FOOT, BranchEnum::RF_LEG>,
      footToBranchKV<FootEnum::LH_FOOT, BranchEnum::LH_LEG>, footToBranchKV<FootEnum::RH_FOOT, BranchEnum::RH_LEG>>;

  template <FootEnum foot, ContactEnum contact>
  using footToContactKV = std_utils::KeyValuePair<FootEnum, ContactEnum, foot, contact>;
  using mapFootEnumToContactEnum =
      std_utils::CompileTimeMap<FootEnum, ContactEnum, footToContactKV<FootEnum::LF_FOOT, ContactEnum::LF_FOOT>,
                                footToContactKV<FootEnum::RF_FOOT, ContactEnum::RF_FOOT>,
                                footToContactKV<FootEnum::LH_FOOT, ContactEnum::LH_FOOT>,
                                footToContactKV<FootEnum::RH_FOOT, ContactEnum::RH_FOOT>>;

  template <BranchEnum branch, BodyEnum body>
  using branchToBodyKV = std_utils::KeyValuePair<BranchEnum, BodyEnum, branch, body>;

  using mapBranchToStartBody =
      std_utils::CompileTimeMap<BranchEnum, BodyEnum, branchToBodyKV<BranchEnum::BASE, BodyEnum::BASE>,
                                branchToBodyKV<BranchEnum::LF_LEG, BodyEnum::LF_HIP>, branchToBodyKV<BranchEnum::RF_LEG, BodyEnum::RF_HIP>,
                                branchToBodyKV<BranchEnum::LH_LEG, BodyEnum::LH_HIP>, branchToBodyKV<BranchEnum::RH_LEG, BodyEnum::RH_HIP>>;

  using mapBranchToEndBody = std_utils::CompileTimeMap<
      BranchEnum, BodyEnum, branchToBodyKV<BranchEnum::BASE, BodyEnum::BASE>, branchToBodyKV<BranchEnum::LF_LEG, BodyEnum::LF_FOOT>,
      branchToBodyKV<BranchEnum::RF_LEG, BodyEnum::RF_FOOT>, branchToBodyKV<BranchEnum::LH_LEG, BodyEnum::LH_FOOT>,
      branchToBodyKV<BranchEnum::RH_LEG, BodyEnum::RH_FOOT>>;

  template <LimbEnum limb, unsigned int uint>
  using limbToNDofKV = std_utils::KeyValuePair<LimbEnum, unsigned int, limb, uint>;
  using mapLimbToNDof =
      std_utils::CompileTimeMap<AnymalTopology::LimbEnum, unsigned int, limbToNDofKV<LimbEnum::LF_LEG, 3u>,
                                limbToNDofKV<LimbEnum::RF_LEG, 3u>, limbToNDofKV<LimbEnum::LH_LEG, 3u>, limbToNDofKV<LimbEnum::RH_LEG, 3u>>;

  using mapLimbToStartIndexInJ =
      std_utils::CompileTimeMap<LimbEnum, unsigned int, limbToNDofKV<LimbEnum::LF_LEG, 0u>, limbToNDofKV<LimbEnum::RF_LEG, 3u>,
                                limbToNDofKV<LimbEnum::LH_LEG, 6u>, limbToNDofKV<LimbEnum::RH_LEG, 9u>>;

  template <BranchEnum branch, unsigned int uint>
  using branchToUIntKV = std_utils::KeyValuePair<BranchEnum, unsigned int, branch, uint>;

  using mapBranchToStartIndexInU =
      std_utils::CompileTimeMap<BranchEnum, unsigned int, branchToUIntKV<BranchEnum::BASE, 0u>, branchToUIntKV<BranchEnum::LF_LEG, 6u>,
                                branchToUIntKV<BranchEnum::RF_LEG, 9u>, branchToUIntKV<BranchEnum::LH_LEG, 12u>,
                                branchToUIntKV<BranchEnum::RH_LEG, 15u>>;
};

}  // namespace anymal_description
