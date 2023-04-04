/*
 * TextLineWidget.hpp
 *
 *  Created on: 3 Jul 2015
 *      Author: markuszahner
 */

#pragma once

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <hri_user_interface/LineWidget.h>

namespace hri_user_interface {
  class StatusLineWidget : public LineWidget {
    std::string text_;
    
    boost::function<void(StatusLineWidget&)> selectFun_;
    boost::function<void(StatusLineWidget&)> focusFun_;
    boost::function<void(StatusLineWidget&)> updateFun_;

  public:
    // the necessary functions
    StatusLineWidget();
    virtual ~StatusLineWidget();

    bool setCurrentText(const std::string& text);
    std::string getCurrentText() const;
    void setSelectFun(const boost::function<void(StatusLineWidget&)> fun);
    void setUpdateFun(const boost::function<void(StatusLineWidget&)> fun);
    void setFocusFun(const boost::function<void(StatusLineWidget&)> fun);

    void update();
    void focus();
    void left();
    void right();
    void select();
    void changeValue(int steps);    
    
    void clear();
  };
};
