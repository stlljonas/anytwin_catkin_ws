/*
 * Form.cpp
 *
 *  Created on: 9 Jun 2015
 *      Author: markuszahner
 */

#include "hri_user_interface/Form.h"

namespace hri_user_interface {

Form::Form()
: cursor_(0)
{}

bool Form::init(const ros::NodeHandle& nh) {
  nh_ = nh;
  return true;
}

int Form::handleButtons(int button, int steps){
  if (button == ButtonConfig::BUTTON_UP) {
    cursorUp();
    lines_[cursor_]->update();
    lines_[cursor_]->focus();
  }
  else if (button == ButtonConfig::BUTTON_DOWN) {
    cursorDown();
    lines_[cursor_]->update();
    lines_[cursor_]->focus();
  }

  if (cursor_ == 0) {
    if (button == ButtonConfig::BUTTON_RIGHT) {
      return 1;
    }
    else if (button == ButtonConfig::BUTTON_LEFT) {
      return -1;
    }
  }
  else {
  if (button == ButtonConfig::BUTTON_RIGHT) {
    lines_[cursor_]->right();
  }
    else if (button == ButtonConfig::BUTTON_LEFT) {
      lines_[cursor_]->left();
    }
    else if (button == ButtonConfig::BUTTON_1) {
      lines_[cursor_]->select();
    }
    else if (button == ButtonConfig::BUTTON_SLIDER_R) {
      lines_[cursor_]->changeValue(steps);
    }
  }
  if (button == ButtonConfig::BUTTON_4) {
    this->button4();
  }
  if (button == ButtonConfig::BUTTON_2) {
    this->button2();
  }
  return 0;
}

std::vector<std::string> Form::print(uint8_t slots){
  // return a vector containing as many
  std::vector<std::string> lines(slots);

  int lowest_line = (int)cursor_-(slots/2);
  lowest_line = std::max(std::min(lowest_line, (int)lines_.size()-slots), 0);

  for (size_t i = 0; i < (size_t)slots; i++) {
    size_t line_to_display = lowest_line+i;
    
    if (line_to_display < lines_.size()) {
      if (cursor_ == line_to_display) {
        if (lines_[line_to_display] != nullptr) {
          lines[i] = ">"+lines_[line_to_display]->getCurrentText();
        } else {
          lines[i] = "";
        }
      }
      else {
        if (lines_[line_to_display] != nullptr) {
          lines[i] = " "+lines_[line_to_display]->getCurrentText();
        } else {
          lines[i] = "";
        }
      }
    } else {
      lines[i] = "";
    }
  }
  
  return lines;
}

void Form::cursorUp() {
  if (cursor_ == 0) {
    if (!lines_.empty())
      cursor_ = lines_.size()-1;
  }
  else {
    --cursor_;
  }
}

void Form::cursorDown() {
  ++cursor_;
  
  if (cursor_ >= lines_.size()) {
    cursor_ = 0;
  }
}

void Form::button4() {
  ROS_DEBUG("[HRI] pressed button 4.");
}

void Form::button2() {
  ROS_DEBUG("[HRI] pressed button 2.");
}


}
