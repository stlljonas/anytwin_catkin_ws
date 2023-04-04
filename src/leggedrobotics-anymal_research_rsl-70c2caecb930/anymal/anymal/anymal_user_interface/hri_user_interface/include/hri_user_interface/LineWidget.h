/*
 * LineWidget.hpp
 *
 *  Created on: 3 Jul 2015
 *      Author: markuszahner
 */

#ifndef HRI_USER_INTERFACE_LINE_WIDGET_H
#define HRI_USER_INTERFACE_LINE_WIDGET_H

#include <string>

#include <boost/shared_ptr.hpp>

namespace hri_user_interface {
  enum ButtonConfig {
    BUTTON_DOWN = 0,
    BUTTON_RIGHT,
    BUTTON_UP,
    BUTTON_LEFT,
    BUTTON_1,
    BUTTON_2,
    BUTTON_3,
    BUTTON_4,
    BUTTON_SLIDER_R, // fake buttons we can use to detect whether a slider has been moved...
    BUTTON_SLIDER_L
  };

  class LineWidget {
  public:
    LineWidget();
    virtual ~LineWidget();
    
    virtual std::string getCurrentText() const;
    
    virtual void update() = 0;                  // update value or property
    virtual void focus() = 0;                   // it just got into focus
    virtual void left() = 0;                    // left button press
    virtual void right() = 0;                   // right button press
    virtual void select() = 0;                  // we've been selected    
    virtual void changeValue(int steps) = 0;    // change value if necessary
  };
  
  typedef boost::shared_ptr<LineWidget> LineWidgetPtr;
};

#endif /* HRI_USER_INTERFACE_LINE_WIDGET_H */
