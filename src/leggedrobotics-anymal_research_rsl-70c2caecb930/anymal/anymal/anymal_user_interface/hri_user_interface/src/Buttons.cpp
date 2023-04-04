/*
 * Buttons.cpp
 *
 *  Created on: 18 May 2015
 *      Author: markuszahner
 */

#include "hri_user_interface/Buttons.h"

namespace hri_user_interface {

Buttons::Buttons() {
  callback_function = NULL;
  
  for (int i = 0; i < 32; i++) {
    button_states[i].last_action.nsec = 0;
    button_states[i].last_action.sec = 0;
    button_states[i].state = 0;
  }
}

void Buttons::init(boost::function<void(int, int)> callback) {
  callback_function = callback;
}

void Buttons::callback(int button_index, int button_event) {
  if (callback_function != NULL) {
    callback_function(button_index, button_event);
  }
}

void Buttons::update(uint in) {
  for (uint i = 0; i < 32; i++) {
    BState & bs = button_states[i];
    
    if (in & 1 << i) { // check bitwise button pressed
      if (bs.state == ButtonState::ZERO) {
        bs.state++;
        bs.last_action = ros::Time::now(); // clicked, save the time
        
        callback(i, ButtonEvent::CLICKED);
      }
      else if ((bs.state == ButtonState::DOWN1 || bs.state == ButtonState::KEPT_DOWN) &&
               ((ros::Time::now()-bs.last_action).toSec() > MULTIPLE_TRIGGER_TIME_S)) {
        bs.state = ButtonState::KEPT_DOWN;
        bs.last_action = ros::Time::now();
      
        callback(i, ButtonEvent::CLICKED);
      }
      else if ((bs.state == ButtonState::UP1) &&
               ((ros::Time::now()-bs.last_action).toSec() < DOUBLE_CLICK_TIME_S)) {
        // this is the second click within short time
        bs.state++;
      }
    }
    else { // button released
      if (bs.state == ButtonState::DOWN1) {
        bs.state ++;
      }
      else if ((bs.state == ButtonState::DOWN2) &&
               ((ros::Time::now()-bs.last_action).toSec() < DOUBLE_CLICK_TIME_S)) {
        bs.state = ButtonState::ZERO; // reset button state
        callback(i,ButtonEvent::DOUBLE_CLICKED); // fire double clicked
      }
      else if ((bs.state == ButtonState::DOWN2) &&
               ((ros::Time::now()-bs.last_action).toSec() > DOUBLE_CLICK_TIME_S)) {
        bs.state = ButtonState::ZERO; // reset button state
      }
      else if ((bs.state == ButtonState::UP1) &&
               ((ros::Time::now()-bs.last_action).toSec() > DOUBLE_CLICK_TIME_S)) {
        bs.state = ButtonState::ZERO; // reset button state
      }
      else if (bs.state == ButtonState::KEPT_DOWN){
        bs.state = ButtonState::ZERO; // reset button state
      }
    }
  }
}

}
