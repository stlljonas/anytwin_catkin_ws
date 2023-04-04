/******************************************************************************
 * Authors:                                                                   *
 *    C. Dario Bellicoso                                                      *
 *    Samuel Bachmann <sbachmann@anybotics.com>                               *
 ******************************************************************************/

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QGridLayout>

#include "rqt_loco_visualizer/LocoVisualizerPlugin.h"
#include "rqt_loco_visualizer/GaitPatternContainer.h"
#include "rqt_loco_visualizer/GaitPatternWidget.h"

namespace rqt_loco_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

LocoVisualizerPlugin::LocoVisualizerPlugin()
    : rqt_gui_cpp::Plugin(),
      widget_(nullptr) {
  // Give QObjects reasonable names.
  setObjectName("LocoVisualizerPlugin");
}

/* ========================================================================== */
/* Initialize/Shutdown                                                        */
/* ========================================================================== */

void LocoVisualizerPlugin::initPlugin(qt_gui_cpp::PluginContext &context) {
  // access standalone command line arguments
  QStringList argv = context.argv();
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

  connect(ui_.lineEditTopic, SIGNAL(textChanged(QString)),
          this, SLOT(lineEditTopicTextChanged(QString)));
}

void LocoVisualizerPlugin::shutdownPlugin() {
  gaitPatternSubscriber_.shutdown();
}

/* ========================================================================== */
/* Settings                                                                   */
/* ========================================================================== */

void LocoVisualizerPlugin::saveSettings(
    qt_gui_cpp::Settings &plugin_settings,
    qt_gui_cpp::Settings &instance_settings) const {
  ROS_DEBUG_STREAM("Saving gait pattern topic name: "
                       << ui_.lineEditTopic->displayText().toStdString());
  plugin_settings.setValue("topic", ui_.lineEditTopic->displayText());
}

void LocoVisualizerPlugin::restoreSettings(
    const qt_gui_cpp::Settings &plugin_settings,
    const qt_gui_cpp::Settings &instance_settings) {
  QString topic = plugin_settings.value("topic").toString();
  if (topic.isEmpty()) {
    topic = QString("/loco_ros/gait_patterns");
  }
  ROS_DEBUG_STREAM("Restore gait pattern topic name: " << topic.toStdString());
  ui_.lineEditTopic->setText(topic);
}

/* ========================================================================== */
/* Callbacks                                                                  */
/* ========================================================================== */

void LocoVisualizerPlugin::gaitInfoCallback(
    const anymal_msgs::GaitPatternsConstPtr &msg) {

  std::vector<GaitPatternContainer> container(msg->patterns.size(),
                                              GaitPatternContainer());

  ui_.widgetGaitPattern->setStridePhase(msg->phase);

  for (int i = 0; i < msg->patterns.size(); i++) {
    for (int k = 0; k < 4; k++) {
      container[i].setDuration(
          msg->patterns[i].duration);
      container[i].setLiftOffPhase(
          msg->patterns[i].liftoff_phases.at(k), k);
      container[i].setTouchDownPhase(
          msg->patterns[i].touchdown_phases.at(k), k);
    }
  }

  ui_.widgetGaitPattern->setGaitContainer(container);
}

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void LocoVisualizerPlugin::lineEditTopicTextChanged(const QString &text) {
  try {
    gaitPatternSubscriber_ = getNodeHandle().subscribe(
        text.toStdString(), 0, &LocoVisualizerPlugin::gaitInfoCallback, this);
  } catch (std::exception &e) {
    ROS_ERROR_STREAM(e.what());
  }
}

PLUGINLIB_EXPORT_CLASS(LocoVisualizerPlugin, rqt_gui_cpp::Plugin)

} // namespace

