#include <QDate>
#include <QMessageBox>

#include <pluginlib/class_list_macros.h>

#include <average_calculator/AverageCalculator.hpp>
#include <message_logger/message_logger.hpp>

#include "rqt_average_calculator/AverageCalculatorPlugin.h"

PLUGINLIB_EXPORT_CLASS(rqt_average_calculator::AverageCalculatorPlugin, rqt_gui_cpp::Plugin)

namespace rqt_average_calculator {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

AverageCalculatorPlugin::AverageCalculatorPlugin() : rqt_gui_cpp::Plugin(), widget_(nullptr) {  // NOLINT
  setObjectName("AverageCalculatorPlugin");
}

/* ========================================================================== */
/* Initialize/Shutdown                                                        */
/* ========================================================================== */

void AverageCalculatorPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // Input validation. Allow integer and floating point numbers separated by a space.
  QRegExp numberRegex(R"(^(\s*-?\d+(\.\d+)?)(\s* \s*-?\d+(\.\d+)?)*\s*$)");
  numberValidator_.setRegExp(numberRegex);
  ui_.lineEditNumbers->setValidator(&numberValidator_);

  // Input placeholder.
  ui_.lineEditNumbers->setPlaceholderText("Space separated list of numbers");

  // Connect.
  connect(ui_.pushButtonAverage, &QPushButton::clicked, this, &AverageCalculatorPlugin::onPushButtonAverageClicked);
  connect(ui_.pushButtonClear, &QPushButton::clicked, this, &AverageCalculatorPlugin::onPushButtonClearClicked);
  connect(ui_.lineEditNumbers, &QLineEdit::textChanged, this, &AverageCalculatorPlugin::onLineEditNumbersTextChanged);
}

void AverageCalculatorPlugin::shutdownPlugin() {
  // Shutdown.
}

/* ========================================================================== */
/* Settings                                                                   */
/* ========================================================================== */

void AverageCalculatorPlugin::saveSettings(qt_gui_cpp::Settings& /*plugin_settings*/, qt_gui_cpp::Settings& /*instance_settings*/) const {
  // Save settings.
  MELO_INFO_STREAM("Save settings.")
}

void AverageCalculatorPlugin::restoreSettings(const qt_gui_cpp::Settings& /*plugin_settings*/,
                                              const qt_gui_cpp::Settings& /*instance_settings*/) {
  // Restore settings.
  MELO_INFO_STREAM("Restore settings.")
}

bool AverageCalculatorPlugin::hasConfiguration() const {
  // To enable the rqt configuration button return true.
  return false;
}

void AverageCalculatorPlugin::triggerConfiguration() {
  // Do something (e.g. open a configuration dialog).
  MELO_INFO_STREAM("Open configuration dialog.")
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

/* ========================================================================== */
/* Callbacks                                                                  */
/* ========================================================================== */

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void AverageCalculatorPlugin::onPushButtonAverageClicked() {
  QStringList list = ui_.lineEditNumbers->text().split(" ", QString::SplitBehavior::SkipEmptyParts);
  if (list.empty()) {
    ui_.labelAverage->setText("undefined");
    return;
  }
  // Calculate average.
  average_calculator::AverageCalculator averageCalculator;
  for (const auto& item : list) {
    bool ok;
    double value = item.toDouble(&ok);
    if (ok) {
      averageCalculator.addValue(value);
    }
  }
  ui_.labelAverage->setText(ui_.labelAverage->text() + QString::number(averageCalculator.getAverageValue()));

  // Surprise.
  QDate today = QDateTime::currentDateTime().date();
  if (averageCalculator.getAverageValue() == 42.0 || (today.month() == 4 && today.day() == 1) ||
      (today.month() == 9 && today.day() == 14)) {
    QMessageBox messageBox;
    messageBox.setWindowTitle("Surprise!");
    messageBox.setText("You won!");
    messageBox.setIconPixmap(QPixmap(":/images/surprise.png"));
    messageBox.exec();
  }
}

void AverageCalculatorPlugin::onPushButtonClearClicked() {
  ui_.labelAverage->setText("");
  ui_.lineEditNumbers->setText("");
}

void AverageCalculatorPlugin::onLineEditNumbersTextChanged(const QString& text) {
  QStringList list = text.split(" ", QString::SplitBehavior::SkipEmptyParts);
  QString average;
  for (int j = 0; j < list.size(); ++j) {
    average += list.at(j);
    if (j < list.size() - 1) {
      average += " + ";
    } else {
      average += " = ";
    }
  }
  ui_.labelAverage->setText(average);
}

}  // namespace rqt_average_calculator
