/*
 * TextLineWidget.hpp
 *
 *  Created on: 3 Jul 2015
 *      Author: markuszahner
 */

#ifndef HRI_USER_INTERFACE_TEXT_LINE_WIDGET_H
#define HRI_USER_INTERFACE_TEXT_LINE_WIDGET_H

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <vector>

#include <hri_user_interface/LineWidget.h>

namespace hri_user_interface {
  class TextLineWidget : public LineWidget {
    std::vector<std::string> lines_;
    int currentLine_;
    
    boost::function<void(TextLineWidget&)> selectFun_;
    boost::function<void(TextLineWidget&)> focusFun_;
    boost::function<void(TextLineWidget&)> updateFun_;

  public:
    // the necessary functions
    TextLineWidget();
    TextLineWidget(const std::string& name);
    virtual ~TextLineWidget();

    void setLines(const std::vector<std::string>& lines);
    bool setCurrentText(const std::string& text);
    std::string getCurrentText() const;
    void setSelectFun(const boost::function<void(TextLineWidget&)> fun);
    void setUpdateFun(const boost::function<void(TextLineWidget&)> fun);
    void setFocusFun(const boost::function<void(TextLineWidget&)> fun);

    void update();
    void focus();
    void left();
    void right();
    void select();
    void changeValue(int steps);    
    
    void clear();
  };
};

#endif /* HRI_USER_INTERFACE_TEXT_LINE_WIDGET_H */
