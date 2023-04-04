/*
 * StatusLineWidget.cpp
 *
 *  Created on: 3 Jul 2015
 *      Author: markuszahner
 */

#include "hri_user_interface/StatusLineWidget.h"

namespace hri_user_interface {

StatusLineWidget::StatusLineWidget() :
    text_() {
}

StatusLineWidget::~StatusLineWidget() {
}

bool StatusLineWidget::setCurrentText(const std::string& text) {
  text_ = text;
  return true;
}

std::string StatusLineWidget::getCurrentText() const {
  return text_;
}

void StatusLineWidget::setSelectFun(boost::function<void(StatusLineWidget&)> fun) {
  selectFun_ = fun;
}

void StatusLineWidget::setUpdateFun(boost::function<void(StatusLineWidget&)> fun) {
  updateFun_ = fun;
}

void StatusLineWidget::setFocusFun(boost::function<void(StatusLineWidget&)> fun) {
  focusFun_ = fun;
}

void StatusLineWidget::update(){
  if (updateFun_ != NULL) {
    updateFun_(*this);
  }
}

void StatusLineWidget::focus() {
  if (focusFun_ != NULL) {
    focusFun_(*this);
  }
}

void StatusLineWidget::left() {
  update();
}

void StatusLineWidget::right() {
  update();
}

void StatusLineWidget::select() {
  update();
}

void StatusLineWidget::changeValue(int steps) {
}

void StatusLineWidget::clear() {
  text_ = "";
}

}

