/******************************************************************************
 * Authors:                                                                   *
 *    C. Dario Bellicoso                                                      *
 *    Samuel Bachmann <sbachmann@anybotics.com>                               *
 ******************************************************************************/

#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <ui_LocoVisualizerPlugin.h>
#include <QWidget>

#include <ros/ros.h>
#include <anymal_msgs/GaitPatterns.h>

namespace rqt_loco_visualizer {

class LocoVisualizerPlugin : public rqt_gui_cpp::Plugin {
Q_OBJECT

public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  LocoVisualizerPlugin();

  /* ======================================================================== */
  /* Initialize/Shutdown                                                      */
  /* ======================================================================== */

  void initPlugin(qt_gui_cpp::PluginContext &context) override;

  void shutdownPlugin() override;

  /* ======================================================================== */
  /* Settings                                                                 */
  /* ======================================================================== */

  void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                    qt_gui_cpp::Settings &instance_settings) const override;

  void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                       const qt_gui_cpp::Settings &instance_settings) override;

private:

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::LocoVisualizer ui_;
  QWidget *widget_;

  ros::Subscriber gaitPatternSubscriber_;

  /* ======================================================================== */
  /* Callbacks                                                                */
  /* ======================================================================== */

  void gaitInfoCallback(const anymal_msgs::GaitPatternsConstPtr &msg);

protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  virtual void lineEditTopicTextChanged(const QString &text);

};

} // namespace
