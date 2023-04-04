/*!
 * @file    anymal_state_estimator_lwf_node.cpp
 * @author  Markus Staeuble
 * @date    May, 2018
 */

#include <any_node/any_node.hpp>
#include <anymal_state_estimator/anymal_state_estimator_basic/AnymalStateEstimatorBasic.hpp>
#include <anymal_state_estimator_lwf/anymal_filter_lwf/AnymalFilterLwf.hpp>

int main(int argc, char** argv) {
  using AnymalFilterLwf = anymal_state_estimator_lwf::AnymalFilterLwf<anymal_description::ConcreteAnymalDescription,
                                                      anymal_model::AnymalState,
                                                      anymal_state_estimator_lwf::KinematicsModelLwf>;

  any_node::Nodewrap<anymal_state_estimator::AnymalStateEstimatorBasic<AnymalFilterLwf>> node(argc, argv, "anymal_state_estimator_lwf", 2);
  return static_cast<int>(!node.execute());
}
