/*
 * Form.h
 *
 *  Created on: 9 Jun 2015
 *      Author: markuszahner
 */

#ifndef HRI_USER_INTERFACE_FORM_H
#define HRI_USER_INTERFACE_FORM_H

#include <ros/ros.h>

#include <hri_user_interface/LineWidget.h>

namespace hri_user_interface {


class Form
{
public:
  /** cursor selects the line within the form
    * cursor = 0; is the top row --> the "name of the Form itself"
    */
  size_t cursor_;
  std::vector<LineWidgetPtr> lines_;
  ros::NodeHandle nh_;

  // methods
  Form();
  virtual ~Form() = default;

  virtual bool init(const ros::NodeHandle& nh);
  virtual int handleButtons(int button, int steps);

  void cursorUp();
  void cursorDown();
  virtual void button4();
  virtual void button2();
  
  /** prints the correct nr of lines (slots) based on cursor position
    */
  std::vector<std::string> print(uint8_t slots);

};

typedef boost::shared_ptr<Form> FormPtr;


} // namespace hri_user_interface

#endif /* HRI_USER_INTERFACE_FORM_H */
