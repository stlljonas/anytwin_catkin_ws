
/*!
 * @file    RandomWalkPrediction.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

#pragma once

#include <any_node/Param.hpp>
#include <tsif/residuals/random_walk.h>

namespace tsif {
// residual implementing a random walk
template <typename... Elements>
class RandomWalkPrediction : public RandomWalk<Elements...> {
 public:
  using Base = RandomWalk<Elements...>;
  using Previous = typename Base::Previous;
  using Current = typename Base::Current;
  using MyElementVector = typename Base::MyElementVector;

  RandomWalkPrediction() : Base(){};
  ~RandomWalkPrediction() override = default;

  int Extrapolate(const typename Previous::CRef pre, typename Current::Ref ext, const double& delta_t) {
    _Extrapolate(pre, ext, delta_t);
    return 0;
  }
  template <int C = 0, typename std::enable_if<(C < MyElementVector::kN)>::type* = nullptr>
  int _Extrapolate(const typename Previous::CRef pre, typename Current::Ref ext, const double& delta_t) {
    const int I = MyElementVector::template GetId<C>();
    // forward state
    ext.template GetElement<I>() = pre.template GetElement<I>();
    _Extrapolate<C + 1>(pre, ext, delta_t);
    return 0;
  }
  template <int C = 0, typename std::enable_if<(C >= MyElementVector::kN)>::type* = nullptr>
  int _Extrapolate(const typename Previous::CRef pre, typename Current::Ref ext, const double& delta_t) {
    return 0;
  }
  int LoadParameters(const ros::NodeHandle& handle, const std::string& id) {
    w_ = param_io::param<double>(handle, id + std::string{"/w"}, 1.);
    return 0;
  }

 protected:
  using Base::w_;
};
}  // namespace tsif
