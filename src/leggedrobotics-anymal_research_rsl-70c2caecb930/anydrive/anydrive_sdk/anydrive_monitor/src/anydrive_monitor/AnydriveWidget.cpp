#include "anydrive_monitor/AnydriveWidget.h"
#include "anydrive_monitor/ui_AnydriveWidget.h"

namespace anydrive_monitor {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

AnydriveWidget::AnydriveWidget(QWidget* parent) : QWidget(parent), ui_(new Ui::AnydriveWidget) {
  ui_->setupUi(this);

  // Reset.
  reset();
}

AnydriveWidget::~AnydriveWidget() {
  delete ui_;
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

Ui::AnydriveWidget* AnydriveWidget::ui() {
  return ui_;
}

void AnydriveWidget::reset() {
  ui_->labelName->setText("");
  ui_->labelState->setText("");
  ui_->labelMode->setText("");
  ui_->labelStatus->setText("");
  ui_->labelPosition->setText("");
  ui_->labelVelocity->setText("");
  ui_->labelTorque->setText("");
  ui_->labelCurrent->setText("");
  ui_->labelVoltage->setText("");
  ui_->labelTemperature->setText("");
}

void AnydriveWidget::hideFeedback() {
  ui_->labelPosition->hide();
  ui_->doubleSpinBoxPosition->hide();
  ui_->labelVelocity->hide();
  ui_->doubleSpinBoxVelocity->hide();
  ui_->labelTorque->hide();
  ui_->doubleSpinBoxTorque->hide();
  ui_->labelCurrent->hide();
  ui_->doubleSpinBoxCurrent->hide();
  ui_->labelVoltage->hide();
  ui_->labelTemperature->hide();

  ui_->labelExpandFeedback1->setText("...");
  ui_->labelExpandFeedback2->setText("...");
}

void AnydriveWidget::showFeedback() {
  ui_->labelPosition->show();
  ui_->doubleSpinBoxPosition->show();
  ui_->labelVelocity->show();
  ui_->doubleSpinBoxVelocity->show();
  ui_->labelTorque->show();
  ui_->doubleSpinBoxTorque->show();
  ui_->labelCurrent->show();
  ui_->doubleSpinBoxCurrent->show();
  ui_->labelVoltage->show();
  ui_->labelTemperature->show();

  ui_->labelExpandFeedback1->setText("");
  ui_->labelExpandFeedback2->setText("");
}

void AnydriveWidget::hideParameters() {
  ui_->doubleSpinBoxParamA->hide();
  ui_->doubleSpinBoxParamB->hide();
  ui_->doubleSpinBoxParamC->hide();

  ui_->labelExpandParameters1->setText("...");
  ui_->labelExpandParameters2->setText("...");
}

void AnydriveWidget::showParameters() {
  ui_->doubleSpinBoxParamA->show();
  ui_->doubleSpinBoxParamB->show();
  ui_->doubleSpinBoxParamC->show();

  ui_->labelExpandParameters1->setText("");
  ui_->labelExpandParameters2->setText("");
}

void AnydriveWidget::hideCommand() {
  ui_->pushButtonResetCommand->hide();
  ui_->pushButtonSendCommand->hide();
  ui_->pushButtonSendCommandDisable->hide();
  ui_->pushButtonSendCommandFreeze->hide();

  ui_->labelExpandCommand1->setText("...");
  ui_->labelExpandCommand2->setText("...");
}

void AnydriveWidget::showCommand() {
  ui_->pushButtonResetCommand->show();
  ui_->pushButtonSendCommand->show();
  ui_->pushButtonSendCommandDisable->show();
  ui_->pushButtonSendCommandFreeze->show();

  ui_->labelExpandCommand1->setText("");
  ui_->labelExpandCommand2->setText("");
}

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void AnydriveWidget::onNameUpdate(QString name) {
  ui_->labelName->setText(name);
}

void AnydriveWidget::onStateUpdate(QString state) {
  ui_->labelState->setText(state);
}

void AnydriveWidget::onModeUpdate(QString mode) {
  ui_->labelMode->setText(mode);
}

void AnydriveWidget::onPositionUpdate(double position) {
  ui_->labelPosition->setText(QString::number(position, 'f', 5));
}

void AnydriveWidget::onVelocityUpdate(double velocity) {
  ui_->labelVelocity->setText(QString::number(velocity, 'f', 5));
}

void AnydriveWidget::onTorqueUpdate(double torque) {
  ui_->labelTorque->setText(QString::number(torque, 'f', 5));
}

void AnydriveWidget::onCurrentUpdate(double current) {
  ui_->labelCurrent->setText(QString::number(current, 'f', 3));
}

void AnydriveWidget::onVoltageUpdate(double voltage) {
  ui_->labelVoltage->setText(QString::number(voltage, 'f', 2));
}

void AnydriveWidget::onTemperatureUpdate(double temperature) {
  ui_->labelTemperature->setText(QString::number(temperature, 'f', 2));
}

}  // namespace anydrive_monitor
