/*!
 * @file    AnymalStateEstimatorBasicTsif.cpp
 * @author  Markus Staeuble
 * @date    Apr, 2018
 */

#include <anymal_state_estimator/anymal_state_estimator_basic/AnymalStateEstimatorBasic.tpp>
#include <anymal_state_estimator_tsif/tsif/ImuKinAnymalOdometryTsif.hpp>

#include <anymal_description/AnymalDescription.hpp>
#include <anymal_model/AnymalModel.hpp>
#include <anymal_state_estimator_tsif/anymal_filter_tsif/AnymalFilterTsif.hpp>

namespace anymal_state_estimator_tsif_default {
namespace internal {
  using AnymalFilterTsif = anymal_state_estimator_tsif::AnymalFilterTsif<anymal_description::ConcreteAnymalDescription,
                                                          				 anymal_model::AnymalState,
                                                          				 tsif::ImuKinAnymalOdometryTsif>;
}
}

template class anymal_state_estimator::AnymalStateEstimatorBasic<anymal_state_estimator_tsif_default::internal::AnymalFilterTsif>;
