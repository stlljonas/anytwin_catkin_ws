#pragma once

#include <string>

#include <QRegExpValidator>
#include <QString>
#include <QWidget>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

#include <rqt_average_calculator/ui_AverageCalculatorPlugin.h>

namespace rqt_average_calculator {

class AverageCalculatorPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT

 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  AverageCalculatorPlugin();

  /* ======================================================================== */
  /* Initialize/Shutdown                                                      */
  /* ======================================================================== */

  void initPlugin(qt_gui_cpp::PluginContext& context) override;

  void shutdownPlugin() override;

  /* ======================================================================== */
  /* Settings                                                                 */
  /* ======================================================================== */

  void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;

  void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

  bool hasConfiguration() const override;

  void triggerConfiguration() override;

  /* ======================================================================== */
  /* Events                                                                   */
  /* ======================================================================== */

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

 private:
  /* ======================================================================== */
  /* Constants                                                                */
  /* ======================================================================== */

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::AverageCalculatorPlugin ui_;
  QWidget* widget_;

  QRegExpValidator numberValidator_;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  /* ======================================================================== */
  /* Callbacks                                                                */
  /* ======================================================================== */

 protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  void onPushButtonAverageClicked();

  void onPushButtonClearClicked();

  void onLineEditNumbersTextChanged(const QString& text);

 signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */
};

}  // namespace rqt_average_calculator
