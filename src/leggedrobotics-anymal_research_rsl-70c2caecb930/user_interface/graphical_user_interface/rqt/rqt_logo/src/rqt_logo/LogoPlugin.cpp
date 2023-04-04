#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QMouseEvent>
#include <QMovie>
#include <QPainter>

#include <pluginlib/class_list_macros.h>
#include <ratio_layouted_frame/RatioLayoutedFrame.h>

#include "rqt_logo/LogoPlugin.h"

PLUGINLIB_EXPORT_CLASS(rqt_logo::LogoPlugin, rqt_gui_cpp::Plugin)

namespace rqt_logo {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

LogoPlugin::LogoPlugin() : rqt_gui_cpp::Plugin(), widget_(nullptr) {  // NOLINT
  setObjectName("LogoPlugin");
}

/* ========================================================================== */
/* Initialize                                                                 */
/* ========================================================================== */

void LogoPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  ui_.frameLogo->setContextMenuPolicy(Qt::CustomContextMenu);

  // Connect.
  connect(ui_.frameLogo, &QWidget::customContextMenuRequested, this, &LogoPlugin::onShowPreviewContextMenu);

  //! Set application icon.
  QApplication::setWindowIcon(QPixmap(":/images/anybotics.ico"));
}

void LogoPlugin::shutdownPlugin() {}

/* ========================================================================== */
/* Settings                                                                   */
/* ========================================================================== */

void LogoPlugin::saveSettings(qt_gui_cpp::Settings& /*plugin_settings*/, qt_gui_cpp::Settings& instance_settings) const {
  instance_settings.setValue("logo_path", logoPath_);
}

void LogoPlugin::restoreSettings(const qt_gui_cpp::Settings& /*plugin_settings*/, const qt_gui_cpp::Settings& instance_settings) {
  logoPath_ = instance_settings.value("logo_path", "").toString();
  loadImage(logoPath_);
}

/* ========================================================================== */
/* Events                                                                     */
/* ========================================================================== */

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

void LogoPlugin::loadImage(const QString& path) {
  bool loadDefaultLogo = true;
  if (QFileInfo::exists(path)) {
    QImage logo = QImage(path);
    if (!logo.isNull()) {
      logoPath_ = path;
      loadDefaultLogo = false;
      ui_.frameLogo->setImage(logo);
    }
  }

  if (loadDefaultLogo) {
    logoPath_ = "";
    QImage defaultLogo = QImage(":/images/anymal.png");
    ui_.frameLogo->setImage(defaultLogo);
  }

  ui_.frameLogo->updateGeometry();
  ui_.frameLogo->repaint();
}

/* ========================================================================== */
/* Callbacks                                                                  */
/* ========================================================================== */

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void LogoPlugin::onShowPreviewContextMenu(const QPoint& position) {
  auto* frame = qobject_cast<ratio_layouted_frame::RatioLayoutedFrame*>(sender());
  QPoint globalPos = frame->mapToGlobal(position);

  auto* menu = new QMenu("Options");
  auto* editAction = new QAction(tr("Edit"), this);
  auto* resetAction = new QAction(tr("Reset"), this);
  editAction->setIcon(QIcon(":/images/edit.png"));
  resetAction->setIcon(QIcon(":/images/reset.png"));
  menu->addAction(editAction);
  menu->addAction(resetAction);

  QString path;
  QAction* selectedAction = menu->exec(globalPos);
  if (selectedAction == editAction) {
    path = QFileDialog::getOpenFileName(widget_, tr("Open Image"), QDir::homePath(), tr("Image Files (*.png *.jpg)"));
  } else if (selectedAction == resetAction) {
    path = "";
  } else {
    return;
  }

  // Load image.
  loadImage(path);
}

}  // namespace rqt_logo
