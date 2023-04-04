/*!
 * @file    AnymalStateEstimatorTsif.cpp
 * @author  Markus Staeuble
 * @date    Apr, 2018
 */

#include <anymal_state_estimator/AnymalStateEstimator.tpp>
#include <anymal_state_estimator_tsif/tsif/ImuKinAnymalOdometryTsif.hpp>

#include <anymal_description/AnymalDescription.hpp>
#include <anymal_model/AnymalModel.hpp>
#include <anymal_model_ros/AnymalContainersRos.hpp>
#include <anymal_state_estimator_tsif/anymal_filter_tsif/AnymalFilterTsif.hpp>

namespace anymal_state_estimator_tsif_default {
namespace internal {
  using AnymalFilterTsif = anymal_state_estimator_tsif::AnymalFilterTsif<anymal_description::ConcreteAnymalDescription,
                                                          				 anymal_model::AnymalState,
                                                          				 tsif::ImuKinAnymalOdometryTsif>;
}
}

template class anymal_state_estimator::AnymalStateEstimator<anymal_description::ConcreteAnymalDescription,
                                                            anymal_model::AnymalState,
                                                            anymal_model_ros::AnymalContainersRos,
                                                            anymal_state_estimator_tsif_default::internal::AnymalFilterTsif>;
