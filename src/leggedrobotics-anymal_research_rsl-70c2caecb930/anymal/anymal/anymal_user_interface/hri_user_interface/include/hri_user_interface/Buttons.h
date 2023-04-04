/*
 * Buttons.hpp
 *
 *  Created on: 18 May 2015
 *      Author: markuszahner
 */

#ifndef HRI_USER_INTERFACE_BUTTOINS_H
#define HRI_USER_INTERFACE_BUTTOINS_H

#include <cstdlib>
#include <cstdio>
#include <cstdint>

#include <ros/ros.h>

/*
 * This Class handles the Buttons of a Joystick bitwise. It checks every bit of an integer whether it has changed or not
 */
// Parameters later to be exported to a yaml file
#define MULTIPLE_TRIGGER_TIME_S 0.2 //Button will fire again when pressed for 1 more than S seconds
#define DOUBLE_CLICK_TIME_S 0.4

namespace hri_user_interface {
  enum ButtonEvent {
    CLICKED = 1,
    DOUBLE_CLICKED =2
  };
  
  enum ButtonState {
    ZERO=0,
    DOWN1,
    UP1,
    DOWN2,
    UP2,
    KEPT_DOWN,
  };

  class Buttons {
  private:
    boost::function<void(int,int)> callback_function;

    struct BState {
      ros::Time last_action;
      int state;
    };

    BState button_states[32];

  public:
    Buttons();
    
    void init(boost::function<void(int, int)> callback);
    void update(uint in);
    void callback(int button_index, int button_event);
  };
};

#endif /* HRI_USER_INTERFACE_BUTTOINS_H */
