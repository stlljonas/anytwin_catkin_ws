/*!
 * @file 		FilterButter.cpp
 * @author 		Marco Hutter
 * @date		Feb, 2012
 * @version 	1.0
 * @ingroup 	robotTask
 */

// basic filters
#include "basic_filters/FilterButter.hpp"

namespace basic_filters {

FilterButter::FilterButter(int dim, int order) {
  // set dimension of vector tb iltered
  dim_ = dim;

  // first, change order to next even value
  if (order % 2 == 1) {
    order_ = order + 1;
    printf("odd filter order, increased the order +1!\n");
  } else {
    order_ = order;
  }
  isInit_ = false;
  firstCall_ = true;

  //! change size of the coefficient vectors
  aCoeff_.resize(Eigen::NoChange, order_ + 1);
  bCoeff_.resize(Eigen::NoChange, order_ + 1);

  //! initialize input and output values
  inputValues_.resize(dim_, order_ + 1);
  outputValues_.resize(dim_, order_ + 1);
  inputValues_.setZero();
  outputValues_.setZero();
}

FilterButter::~FilterButter() = default;

bool FilterButter::initLowpass(double fs, double fc) {
  // check for correct sampling and cut off frequency
  if (fc > fs / 2) {
    printf("cut-off frequency is too high!\n");
    return false;
  } else {
    // set start coefficients
    aCoeff_.setZero();
    bCoeff_.setZero();
    aCoeff_(0) = 1;
    bCoeff_(0) = 1;

    // create the butterworth filter
    double fr = fs / fc;
    double Oc = tan(M_PI / fr);

    double c;
    Eigen::Matrix<double, 1, 3> a, b;
    Eigen::MatrixXd Amat;
    Amat.resize(order_ + 1, order_ + 1);
    Eigen::MatrixXd Bmat;
    Bmat.resize(order_ + 1, order_ + 1);

    for (int k = 0; k < order_ / 2; k++) {
      c = 1 + 2 * cos(M_PI * (2 * k + 1) / (2 * order_)) * Oc + pow(Oc, 2);
      a(0) = pow(Oc, 2) / c;
      a(2) = a(0);
      a(1) = 2 * a(0);

      b(0) = 1;
      b(1) = 2 * (pow(Oc, 2) - 1) / c;
      b(2) = (1 - 2 * cos(M_PI * (2 * k + 1) / (2 * order_)) * Oc + pow(Oc, 2)) / c;

      Amat.setZero();
      Bmat.setZero();
      for (int j = 0; j < order_ - 1; j++) {
        Amat.block<1, 3>(j, j) = a;
        Bmat.block<1, 3>(j, j) = b;
      }
      // cout << "aCoeff = " << endl << aCoeff_ << endl;
      // cout << "Amat =" << endl << Amat << endl;
      aCoeff_ = aCoeff_ * Amat;
      bCoeff_ = bCoeff_ * Bmat;
      // cout << "aCoeff = " << endl << aCoeff_ << endl;
    }
    isInit_ = true;
    firstCall_ = true;
    return true;
  }
}

Eigen::Matrix<double, Eigen::Dynamic, 1> FilterButter::filterData(const Eigen::Matrix<double, Eigen::Dynamic, 1>& actValues) {
  if (firstCall_) {
    // reset the filter to the first value
    for (int i = 0; i < inputValues_.cols(); i++) {
      inputValues_.col(i) = actValues;
      outputValues_.col(i) = actValues;
    }
    firstCall_ = false;
  }

  if (isInit_) {
    //		printf("filter data\n");
    //! first update the input values
    Eigen::MatrixXd H;
    H = inputValues_.block(0, 0, dim_, order_);
    inputValues_.block(0, 1, dim_, order_) = H;
    inputValues_.block(0, 0, dim_, 1) = actValues;

    //! shift back output values
    H = outputValues_.block(0, 0, dim_, order_);
    outputValues_.block(0, 1, dim_, order_) = H;
    //! apply butter filter and update next output value

    outputValues_.block(0, 0, dim_, 1) =
        1 / bCoeff_(0, 0) *
        (inputValues_ * aCoeff_.transpose() - outputValues_.block(0, 1, dim_, order_) * bCoeff_.block(0, 1, 1, order_).transpose());

    return outputValues_.block(0, 0, dim_, 1);
  } else {
    // just return the input value!
    return actValues;
    // printf("filter needs first to be initialized!\n");
  }
}

bool FilterButter::deactivate() {
  isInit_ = false;
  return true;
}

}  // namespace basic_filters
