/*!
 * @file    AnymalFilterTsif.cpp
 * @author  Markus Staeuble
 * @date    Apr, 2018
 */

#include <anymal_state_estimator_tsif/anymal_filter_tsif/AnymalFilterTsif.tpp>
#include <anymal_state_estimator_tsif/tsif/ImuKinAnymalOdometryTsif.hpp>

#include <anymal_description/AnymalDescription.hpp>
#include <anymal_model/AnymalModel.hpp>

template class anymal_state_estimator_tsif::AnymalFilterTsif<anymal_description::ConcreteAnymalDescription,
                                                             anymal_model::AnymalState,
                                                          	 tsif::ImuKinAnymalOdometryTsif>;
