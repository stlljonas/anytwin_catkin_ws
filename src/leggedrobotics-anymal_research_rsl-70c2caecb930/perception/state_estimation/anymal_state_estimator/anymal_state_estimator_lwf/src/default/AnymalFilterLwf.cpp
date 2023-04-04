/*!
 * @file    AnymalFilterLwf.cpp
 * @author  Markus Staeuble
 * @date    Apr, 2018
 */

#include <anymal_state_estimator_lwf/anymal_filter_lwf/AnymalFilterLwf.tpp>

#include <anymal_description/AnymalDescription.hpp>
#include <anymal_model/AnymalModel.hpp>
#include <anymal_state_estimator_lwf/anymal_filter_lwf/KinematicsModelLwf.hpp>

template class anymal_state_estimator_lwf::AnymalFilterLwf<anymal_description::ConcreteAnymalDescription,
                                                          anymal_model::AnymalState,
                                                          anymal_state_estimator_lwf::KinematicsModelLwf>;
