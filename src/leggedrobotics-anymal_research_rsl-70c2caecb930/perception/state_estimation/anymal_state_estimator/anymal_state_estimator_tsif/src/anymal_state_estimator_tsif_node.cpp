
/*!
 * @file    anymal_state_estimator_tsif_node.cpp
 * @author  Markus Staeuble
 * @date    May, 2018
 */

#include <any_node/any_node.hpp>
#include <anymal_state_estimator/anymal_state_estimator_basic/AnymalStateEstimatorBasic.hpp>
#include <anymal_state_estimator_tsif/anymal_filter_tsif/AnymalFilterTsif.hpp>
#include <anymal_state_estimator_tsif/tsif/ImuKinAnymalOdometryTsif.hpp>

int main(int argc, char** argv) {
  using AnymalFilterTsif = anymal_state_estimator_tsif::AnymalFilterTsif<anymal_description::ConcreteAnymalDescription, anymal_model::AnymalState, tsif::ImuKinAnymalOdometryTsif>;
  any_node::Nodewrap<anymal_state_estimator::AnymalStateEstimatorBasic<AnymalFilterTsif>> node(argc, argv, "anymal_state_estimator_tsif", 2);
  return static_cast<int>(!node.execute());
}
