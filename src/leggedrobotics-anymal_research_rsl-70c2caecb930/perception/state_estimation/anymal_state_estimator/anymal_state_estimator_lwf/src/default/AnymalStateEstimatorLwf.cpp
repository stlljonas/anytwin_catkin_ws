/*!
 * @file    AnymalStateEstimatorLwf.cpp
 * @author  Markus Staeuble
 * @date    Apr, 2018
 */

#include <anymal_state_estimator/AnymalStateEstimator.tpp>

#include <anymal_description/AnymalDescription.hpp>
#include <anymal_model/AnymalModel.hpp>
#include <anymal_model_ros/AnymalContainersRos.hpp>
#include <anymal_state_estimator_lwf/anymal_filter_lwf/KinematicsModelLwf.hpp>
#include <anymal_state_estimator_lwf/anymal_filter_lwf/AnymalFilterLwf.hpp>


namespace anymal_state_estimator_lwf_default {
namespace internal {
  using AnymalFilterLwf = anymal_state_estimator_lwf::AnymalFilterLwf<anymal_description::ConcreteAnymalDescription,
                                                          anymal_model::AnymalState,
                                                          anymal_state_estimator_lwf::KinematicsModelLwf>;
}
}

template class anymal_state_estimator::AnymalStateEstimator<anymal_description::ConcreteAnymalDescription,
                                                            anymal_model::AnymalState,
                                                            anymal_model_ros::AnymalContainersRos,
                                                            anymal_state_estimator_lwf_default::internal::AnymalFilterLwf>;
