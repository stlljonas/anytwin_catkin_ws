
/*!
 * @file    MapLocalizationTsifDefinition.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

// residuals for the map localization tsif
#include <anymal_state_estimator_tsif/tsif/residuals/ExtFrameCentricPoseUpdate.hpp>
#include <anymal_state_estimator_tsif/tsif/residuals/OdomFrameCentricPoseUpdate.hpp>
#include <anymal_state_estimator_tsif/tsif/residuals/RandomWalkPrediction.hpp>

#include <tsif/filter.h>

namespace tsif {

struct MapLocalizationTsifDefinition {
 public:

  MapLocalizationTsifDefinition() = delete;

  enum StateEnum : unsigned int {
    //base pose in odom
    I_R_IB = 0,
    PHI_IB,
    //map frame origin pose in odom
    I_R_IJ,
    PHI_IJ,
    //number of states
    NUM_STATES
  };

  // state indexing for fix parameters. these need to have negative indices!
  enum ParamEnum : int {
    //pose of measurement frame in base
    B_R_BV = -4,
    PHI_BV,
    //identity parameters
    R_NULL,
    PHI_NULL
  };

  //residual indexing. this needs to match the filter typedef!
  enum ResidualEnum : unsigned int {
    //update of the base pose in world
    BASE_POSE_UPD,
    //update of SLAM origin pose in world
    MAP_POSE_UPD,
    //randomwalks for the poses
    BASE_POSE_PRD,
    MAP_POSE_PRD,
  };

  //base type for the loca tsif. this needs to match the residual enum!
  using FilterBase = Filter<ExtFrameCentricPoseUpdate<I_R_IB, PHI_IB, R_NULL, PHI_NULL>,
                            OdomFrameCentricPoseUpdate<I_R_IB, PHI_IB,I_R_IJ, PHI_IJ, B_R_BV, PHI_BV>,
                            RandomWalkPrediction<Element<Vec3,I_R_IB>, Element<Quat,PHI_IB>>,
                            RandomWalkPrediction<Element<Vec3,I_R_IJ>, Element<Quat,PHI_IJ>>>;

};

} /* namespace tsif */
