#pragma once

#include <memory>
#include <string>

#include <QEvent>
#include <QImage>
#include <QLabel>
#include <QMenu>
#include <QSize>
#include <QString>
#include <QVector>
#include <QWidget>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

#include <rqt_logo/ui_LogoPlugin.h>

namespace rqt_logo {

class LogoPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT

 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  LogoPlugin();

  /* ======================================================================== */
  /* Initialize                                                               */
  /* ======================================================================== */

  void initPlugin(qt_gui_cpp::PluginContext& context) override;

  void shutdownPlugin() override;

  /* ======================================================================== */
  /* Settings                                                                 */
  /* ======================================================================== */

  void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;

  void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

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

  Ui::LogoPlugin ui_;
  QWidget* widget_;

  QString logoPath_;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  void loadImage(const QString& path);

  /* ======================================================================== */
  /* Callbacks                                                                */
  /* ======================================================================== */

 protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  void onShowPreviewContextMenu(const QPoint& position);

 signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */
};

}  // namespace rqt_logo
