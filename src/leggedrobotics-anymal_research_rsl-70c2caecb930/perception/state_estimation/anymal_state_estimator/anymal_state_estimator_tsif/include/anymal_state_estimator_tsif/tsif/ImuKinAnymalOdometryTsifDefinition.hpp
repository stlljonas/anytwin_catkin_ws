
/*!
 * @file    ImuKinAnymalOdometryTsifDefinition.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

//residuals for the odom tsif
#include <anymal_state_estimator_tsif/tsif/residuals/AngularVelocityUpdate.hpp>
#include <anymal_state_estimator_tsif/tsif/residuals/AttitudePrediction.hpp>
#include <anymal_state_estimator_tsif/tsif/residuals/LandMarkInOdomUpdate.hpp>
#include <anymal_state_estimator_tsif/tsif/residuals/LinearVelocityPrediction.hpp>
#include <anymal_state_estimator_tsif/tsif/residuals/PositionPrediction.hpp>
#include <anymal_state_estimator_tsif/tsif/residuals/RandomWalkPrediction.hpp>
#include <anymal_state_estimator_tsif/tsif/residuals/TwoWeightRandomWalkPrediction.hpp>

#include <tsif/filter.h>

namespace tsif {

struct ImuKinAnymalOdometryTsifDefinition {
 public:

  ImuKinAnymalOdometryTsifDefinition() = delete;

  enum StateEnum : unsigned int {
    //position base to odom in odom
    I_R_IB = 0,
    //orientation base to odom
    PHI_IB,
    //linear velocity imu to odom in base
    B_V_IM,
    //angular velocity base to odom in base
    B_OMEGA_IB,
    //position right front contact point to odom in odom
    I_P_RF,
    //position left front contact point to odom in odom
    I_P_LF,
    //position left hind contact point to odom in odom
    I_P_LH,
    //position right hind contact point to odom in odom
    I_P_RH,
    //IMU linear acceleration bias in base frame
    B_B_F,
    //IMU angular velocity bias in base frame
    B_B_OMEGA,
    //number of states
    NUM_STATES
  };

  //state indexing for fix parameters. these need to have negative indices!
  enum ParamEnum : int {
    //imu to base position in base frame
    B_R_BM = -1
  };

  //residual indexing, this needs to match the filter base type typedef!
  enum ResidualEnum : unsigned int {
    //prediction of base position from velocity state
    POS_PRD = 0,
    //prediction of base attitude from anugular velocity state
    ATT_PRD,
    //prediction of imu linear velocity from IMU measurements
    LVEL_PRD,
    //update of base angular velocity from IMU measurements
    AVEL_UPD,
    //random walk of the imu linear acceleration bias
    IBF_PRD,
    //random walk of the imu angular velocity bias
    IBO_PRD,
    //two-weight random walks for the contact points
    LMK_PRD_RF,
    LMK_PRD_LF,
    LMK_PRD_LH,
    LMK_PRD_RH,
    //updates comparing contact points as measured with the filter states
    LMK_UPD_RF,
    LMK_UPD_LF,
    LMK_UPD_LH,
    LMK_UPD_RH
  };

  //base type for the odom filter, this needs to match the residual enums!
  using FilterBase = Filter<PositionPrediction<I_R_IB, PHI_IB, B_V_IM, B_OMEGA_IB, B_R_BM>,
                            AttitudePrediction<PHI_IB, B_OMEGA_IB>,
                            LinearVelocityPrediction<PHI_IB, B_V_IM, B_OMEGA_IB, B_B_F>,
                            AngularVelocityUpdate<B_OMEGA_IB, B_B_OMEGA>,
                            RandomWalkPrediction<Element<Vec3, B_B_F>>,
                            RandomWalkPrediction<Element<Vec3, B_B_OMEGA>>,
                            TwoWeightRandomWalkPrediction<Element<Vec3, I_P_RF>>,
                            TwoWeightRandomWalkPrediction<Element<Vec3, I_P_LF>>,
                            TwoWeightRandomWalkPrediction<Element<Vec3, I_P_LH>>,
                            TwoWeightRandomWalkPrediction<Element<Vec3, I_P_RH>>,
                            LandmarkInOdomUpdate<I_R_IB, PHI_IB, I_P_RF>,
                            LandmarkInOdomUpdate<I_R_IB, PHI_IB, I_P_LF>,
                            LandmarkInOdomUpdate<I_R_IB, PHI_IB, I_P_LH>,
                            LandmarkInOdomUpdate<I_R_IB, PHI_IB, I_P_RH>>;

};

} /* namespace tsif */
