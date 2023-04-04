/*
 * ParameterLineWidget.hpp
 *
 *  Created on: 3 Jul 2015
 *      Author: markuszahner
 */

#ifndef HRI_USER_INTERFACE_PARAMETER_LINE_WIDGET_H
#define HRI_USER_INTERFACE_PARAMETER_LINE_WIDGET_H

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <hri_user_interface/LineWidget.h>
#include <parameter_handler/parameter_handler.hpp>

namespace hri_user_interface {
  class ParameterLineWidget : public LineWidget {
    int currentParam_;
    std::vector<parameter_handler::Parameter<double> > params_;
    
    boost::function<void(ParameterLineWidget&)> valueFun_;  // here the value is adjusted and sent to the controller
    boost::function<void(ParameterLineWidget&)> focusFun_;  // here we update the value
    boost::function<void(ParameterLineWidget&)> updateFun_; // here we update the list (called on entry)
    boost::function<void(ParameterLineWidget&)> selectFun_; // if necessary (toggle line) we can use this slot

  public:
    // the necessary functions
    ParameterLineWidget();
    ParameterLineWidget(const std::string& name);
    virtual ~ParameterLineWidget();

    void setParamKeys(const std::vector<std::string>& keys);
    bool setParamValue(const std::string& key,
                       double currentValue,
                       double minValue,
                       double maxValue,
                       double defaultValue);
    bool setCurrentParam(const std::string& key);
    std::string getCurrentParam() const;
    void setCurrentValue(double value);
    double getCurrentValue() const;
    std::string getCurrentText() const;
    void setValueFun(boost::function<void(ParameterLineWidget&)> fun);
    void setUpdateFun(boost::function<void(ParameterLineWidget&)> fun);
    void setFocusFun(boost::function<void(ParameterLineWidget&)> fun);
    void setSelectFun(boost::function<void(ParameterLineWidget&)> fun);
    
    void update();
    void focus();
    void left();
    void right();
    void value(int steps);
    void select();
    void changeValue(int steps);    
    
    void updateCurrentParam(int steps);

    void clear();
  };
};

#endif /* HRI_USER_INTERFACE_PARAMETER_LINE_WIDGET_H */
