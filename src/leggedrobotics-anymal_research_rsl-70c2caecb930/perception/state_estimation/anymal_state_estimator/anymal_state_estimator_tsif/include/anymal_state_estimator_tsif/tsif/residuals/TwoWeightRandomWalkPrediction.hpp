
/*!
 * @file    TwoWeightRandomWalkPrediction.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

#pragma once

#include <any_node/Param.hpp>
#include <tsif/residual.h>

namespace tsif {

// measurement class for a bool
class BoolMeasurement : public ElementVector<Element<bool, 0>> {
 public:
  BoolMeasurement() : ElementVector<Element<bool, 0>>(true){};
  explicit BoolMeasurement(const bool& b) : ElementVector<Element<bool, 0>>(b){};
  const bool& GetMeas() const { return Get<0>(); };
  bool& GetMeas() { return Get<0>(); };
};

// residual implementing a random walk with switchable weights
template <typename... Elements>
using TwoWeightRandomWalkPredictionBase =
    Residual<ElementVector<Element<Vec<Elements::kDim>, Elements::kI>...>, ElementVector<Elements...>,
             ElementVector<Elements...>, BoolMeasurement>;

template <typename... Elements>
class TwoWeightRandomWalkPrediction : public TwoWeightRandomWalkPredictionBase<Elements...> {
 public:
  using MyElementVector = ElementVector<Elements...>;
  using Base = TwoWeightRandomWalkPredictionBase<Elements...>;
  using Base::dt_;
  using Base::meas_;
  using Output = typename Base::Output;
  using Previous = typename Base::Previous;
  using Current = typename Base::Current;

  TwoWeightRandomWalkPrediction() : Base(true, true, true) {}
  ~TwoWeightRandomWalkPrediction() override = default;

  int EvalRes(typename Output::Ref out, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    _EvalRes(out, pre, cur);
    return 0;
  }
  template <int C = 0, typename std::enable_if<(C < MyElementVector::kN)>::type* = nullptr>
  int _EvalRes(typename Output::Ref out, const typename Previous::CRef pre, const typename Current::CRef cur) {
    const int I = MyElementVector::template GetId<C>();
    cur.template GetElement<I>().Boxminus(pre.template GetElement<I>(), out.template Get<I>());
    _EvalRes<C + 1>(out, pre, cur);
    return 0;
  }
  template <int C = 0, typename std::enable_if<(C >= MyElementVector::kN)>::type* = nullptr>
  int _EvalRes(typename Output::Ref /*out*/, const typename Previous::CRef /*pre*/, const typename Current::CRef /*cur*/) {
    return 0;
  }
  int JacPre(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    _JacPre(J, pre, cur);
    return 0;
  }
  template <int C = 0, typename std::enable_if<(C < MyElementVector::kN)>::type* = nullptr>
  int _JacPre(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) {
    const int I = MyElementVector::template GetId<C>();
    this->template SetJacPre<I, I>(J, pre, cur.template GetElement<I>().BoxminusJacRef(pre.template GetElement<I>()));
    _JacPre<C + 1>(J, pre, cur);
    return 0;
  }
  template <int C = 0, typename std::enable_if<(C >= MyElementVector::kN)>::type* = nullptr>
  int _JacPre(MatRefX /*J*/, const typename Previous::CRef /*pre*/, const typename Current::CRef /*cur*/) {
    return 0;
  }
  int JacCur(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    _JacCur(J, pre, cur);
    return 0;
  }
  template <int C = 0, typename std::enable_if<(C < MyElementVector::kN)>::type* = nullptr>
  int _JacCur(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) {
    const int I = MyElementVector::template GetId<C>();
    this->template SetJacCur<I, I>(J, cur, cur.template GetElement<I>().BoxminusJacInp(pre.template GetElement<I>()));
    _JacCur<C + 1>(J, pre, cur);
    return 0;
  }
  template <int C = 0, typename std::enable_if<(C >= MyElementVector::kN)>::type* = nullptr>
  int _JacCur(MatRefX /*J*/, const typename Previous::CRef /*pre*/, const typename Current::CRef /*cur*/) {
    return 0;
  }
  double GetWeight() override {
    // choose weight depending on measurement
    if (meas_->GetMeas()) {return w_true_ / sqrt(dt_); }
    else {return w_false_ / sqrt(dt_); }
  }
  int Extrapolate(const typename Previous::CRef pre, typename Current::Ref ext, const double& delta_t) {
    _Extrapolate(pre, ext, delta_t);
    return 0;
  }
  template <int C = 0, typename std::enable_if<(C < MyElementVector::kN)>::type* = nullptr>
  int _Extrapolate(const typename Previous::CRef pre, typename Current::Ref ext, const double& delta_t) {
    const int I = MyElementVector::template GetId<C>();
    // forward the state
    ext.template GetElement<I>() = pre.template GetElement<I>();
    _Extrapolate<C + 1>(pre, ext, delta_t);
    return 0;
  }
  template <int C = 0, typename std::enable_if<(C >= MyElementVector::kN)>::type* = nullptr>
  int _Extrapolate(const typename Previous::CRef pre, typename Current::Ref ext, const double& delta_t) {
    return 0;
  }
  int LoadParameters(const ros::NodeHandle& handle, const std::string& id) {
    w_true_ = param_io::param<double>(handle, id + std::string{"/w_true"}, 1.);
    w_false_ = param_io::param<double>(handle, id + std::string{"/w_false"}, 1.);
    return 0;
  }

 protected:
  double w_true_{1.};
  double w_false_{1.};
};

}  // namespace tsif
