/*
 * ParameterLineWidget.cpp
 *
 *  Created on: 3 Jul 2015
 *      Author: markuszahner
 */

#include "hri_user_interface/ParameterLineWidget.h"

namespace hri_user_interface {

ParameterLineWidget::ParameterLineWidget() :
  currentParam_(0) {
}

ParameterLineWidget::ParameterLineWidget(const std::string& name) :
  currentParam_(0),
  params_(1) {
  params_[0].setName(name);
}

ParameterLineWidget::~ParameterLineWidget() {
}

void ParameterLineWidget::setParamKeys(const std::vector<std::string>& keys) {
  params_.resize(keys.size());
  for (size_t i = 0; i < keys.size(); i++) {
    params_[i].setName(keys[i]);
  }
}

bool ParameterLineWidget::setParamValue(const std::string& key,
                                        double currentValue,
                                        double minValue,
                                        double maxValue,
                                        double defaultValue) {
  for (size_t i = 0; i < params_.size(); i++) {
    if (params_[i].getName() == key) {
      params_[i].setValue(currentValue);
      params_[i].setMinValue(minValue);
      params_[i].setMaxValue(maxValue);
      params_[i].setDefaultValue(defaultValue);
      
      return true;
    }
  }
  
  return false;
}

bool ParameterLineWidget::setCurrentParam(const std::string& key) {
  for (size_t i =0; i <params_.size(); i++) {
    if (params_[i].getName() == key) {
      currentParam_ = (int)i;
      return true;
    }
  }
  
  return false;
}

std::string ParameterLineWidget::getCurrentParam() const {
  if (params_.size() > 0) {
    return params_[currentParam_].getName();
  }
  else {
    return "none";
  }
}

void ParameterLineWidget::setCurrentValue(double value) {
  params_[currentParam_].setValue(value);
}

double ParameterLineWidget::getCurrentValue() const {
  return params_[currentParam_].getValue();
}

void ParameterLineWidget::setValueFun(boost::function<void(ParameterLineWidget&)> fun) {
  valueFun_ = fun;
}

void ParameterLineWidget::setUpdateFun(boost::function<void(ParameterLineWidget&)> fun) {
  updateFun_ = fun;
}

void ParameterLineWidget::setFocusFun(boost::function<void(ParameterLineWidget&)> fun) {
  focusFun_ = fun;
}

void ParameterLineWidget::setSelectFun(boost::function<void(ParameterLineWidget&)> fun) {
  selectFun_ = fun;
}

std::string ParameterLineWidget::getCurrentText() const {
  if (params_.size() > 0) {
    char num[10];
    int n_num = snprintf(num,10," %.2f",params_[currentParam_].getValue()); // last 6 for name
    
    std::string name = params_[currentParam_].getName();
    name.resize(19,' ');
    name.insert(19-n_num,num);
    name.resize(19);
    
    return name;
  }
  else {
    return LineWidget::getCurrentText();
  }
}

void ParameterLineWidget::update() {
  if (updateFun_ != NULL) {
    updateFun_(*this);
  }
}

void ParameterLineWidget::focus() {
  if (focusFun_ != NULL) {
    focusFun_(*this);
  }
}

void ParameterLineWidget::left() {
  if (params_.size()>1) {
    currentParam_ = (currentParam_-1)%params_.size();
  }
}

void ParameterLineWidget::right() {
  if (params_.size()>0) {
    currentParam_ = (currentParam_+1)%params_.size();
  }
}

void ParameterLineWidget::changeValue(int steps) {
  updateCurrentParam(steps);

  if (valueFun_ != NULL) {
    valueFun_(*this);
  }
}

void ParameterLineWidget::select() {
  if (selectFun_ != NULL) {
    selectFun_(*this);
  }
}

void ParameterLineWidget::updateCurrentParam(int steps) {
  // 1 step = (max-min)/1000
   parameter_handler::Parameter<double> & p = params_[currentParam_];
  
  double toAdd = steps*(p.getMaxValue()-p.getMinValue())/1000.0;
  double min_val = p.getMinValue();
  double max_val = p.getMaxValue();
  double newValue = std::min(std::max(p.getValue()+toAdd,min_val),max_val);
  
  p.setValue(newValue);
}

void ParameterLineWidget::clear() {
  params_.clear();
}

}
