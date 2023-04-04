/*
 * TextLineWidget.cpp
 *
 *  Created on: 3 Jul 2015
 *      Author: markuszahner
 */

#include "hri_user_interface/TextLineWidget.h"

namespace hri_user_interface {

TextLineWidget::TextLineWidget() :
  currentLine_(0) {
}

TextLineWidget::TextLineWidget(const std::string& name) :
    lines_(1, name),
    currentLine_(0) {
}

TextLineWidget::~TextLineWidget() {
}

void TextLineWidget::setLines(const std::vector<std::string>& lines) {
  lines_ = lines;
}

bool TextLineWidget::setCurrentText(const std::string& text) {
  for (size_t i = 0; i < lines_.size(); i++) {
    if (lines_[i] == text) {
      currentLine_ = (size_t)i;
      return true;
    }
  }
  currentLine_ = 0;
  return false;
}

std::string TextLineWidget::getCurrentText() const {
  if (lines_.size() > (size_t)currentLine_) {
    return lines_[currentLine_];
  }
  else {
    return LineWidget::getCurrentText();
  }
}

void TextLineWidget::setSelectFun(boost::function<void(TextLineWidget&)> fun) {
  selectFun_ = fun;
}

void TextLineWidget::setUpdateFun(boost::function<void(TextLineWidget&)> fun) {
  updateFun_ = fun;
}

void TextLineWidget::setFocusFun(boost::function<void(TextLineWidget&)> fun) {
  focusFun_ = fun;
}

void TextLineWidget::update(){
  if (updateFun_ != NULL) {
    updateFun_(*this);
  }
}

void TextLineWidget::focus() {
  if (focusFun_ != NULL) {
    focusFun_(*this);
  }
}

void TextLineWidget::left() {
  if (lines_.size() > 0) {
	if (currentLine_ > 0) {
	  currentLine_ = (currentLine_-1) % lines_.size();
	}
	else {
	  currentLine_ = lines_.size()-1;
	}
  }
}

void TextLineWidget::right() {
  if (lines_.size() > 0) {
    currentLine_ = (currentLine_+1) % lines_.size();
  }
}

void TextLineWidget::select() {
  if (selectFun_ != NULL) {
    selectFun_(*this);
  }
}

void TextLineWidget::changeValue(int steps) {
}

void TextLineWidget::clear() {
  lines_.clear();
}

}

