#include "anydrive_monitor/AnydriveInterface.h"
#include <anydrive/fsm/Controlword.hpp>
#include <cassert>
#include "anydrive_monitor/ui_AnydriveWidget.h"

namespace anydrive_monitor {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

AnydriveInterface::AnydriveInterface(QWidget* parent) : AnydriveWidget(parent) {
  // Initialize state.
  ui_->labelState->setText(QString::fromStdString(anydrive::fsm::stateEnumToName(anydrive::fsm::StateEnum::NA)));

  // Initialize state comboBox.
  ui_->comboBoxState->addItem(QString::fromStdString(anydrive::fsm::stateEnumToName(anydrive::fsm::StateEnum::Configure)));
  ui_->comboBoxState->addItem(QString::fromStdString(anydrive::fsm::stateEnumToName(anydrive::fsm::StateEnum::Calibrate)));
  ui_->comboBoxState->addItem(QString::fromStdString(anydrive::fsm::stateEnumToName(anydrive::fsm::StateEnum::Standby)));
  ui_->comboBoxState->addItem(QString::fromStdString(anydrive::fsm::stateEnumToName(anydrive::fsm::StateEnum::MotorOp)));
  ui_->comboBoxState->addItem(QString::fromStdString(anydrive::fsm::stateEnumToName(anydrive::fsm::StateEnum::ControlOp)));

  // Initialize controlword comboBox.
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_CONFIGURE_TO_CALIBRATE)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_CONFIGURE_TO_STANDBY)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_CALIBRATE_TO_CONFIGURE)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_STANDBY_TO_CONFIGURE)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_STANDBY_TO_MOTOR_PREOP)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_MOTOR_OP_TO_STANDBY)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_MOTOR_OP_TO_CONTROL_OP)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_CONTROL_OP_TO_STANDBY)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_CONTROL_OP_TO_MOTOR_OP)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_CLEAR_ERRORS_TO_MOTOR_OP)));
  ui_->comboBoxControlword->addItem(QString::fromStdString(anydrive::fsm::controlwordIdToString(ANYDRIVE_CW_ID_WARM_RESET)));

  // Initialize mode.
  ui_->labelMode->setText(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::NA)));

  // Initialize mode comboBox.
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::Freeze)));
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::Disable)));
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::Current)));
  //  ui_->comboBoxMode->addItem(QString::fromStdString(
  //      anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::MotorPosition)));
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::MotorVelocity)));
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::GearPosition)));
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::GearVelocity)));
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::JointPosition)));
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::JointVelocity)));
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::JointTorque)));
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::JointPositionVelocity)));
  ui_->comboBoxMode->addItem(QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::JointPositionVelocityTorque)));
  ui_->comboBoxMode->addItem(
      QString::fromStdString(anydrive::mode::modeEnumToName(anydrive::mode::ModeEnum::JointPositionVelocityTorquePidGains)));

  // Initialize error box.
  ui_->labelStatus->setStyleSheet("QLabel { background-color : grey; }");

  // Initialize feedback.
  ui_->labelPosition->setText("N/A");
  ui_->labelVelocity->setText("N/A");
  ui_->labelTorque->setText("N/A");
  ui_->labelCurrent->setText("N/A");
  ui_->labelVoltage->setText("N/A");
  ui_->labelTemperature->setText("N/A");

  // Connect signals, slots.
  connect(ui_->pushButtonSendControlword, SIGNAL(clicked()), this, SLOT(onSendControlword()));
  connect(ui_->pushButtonSendCommand, SIGNAL(clicked()), this, SLOT(onSendCommand()));
  connect(ui_->pushButtonSendCommandDisable, SIGNAL(clicked()), this, SLOT(onSendCommandDisable()));
  connect(ui_->pushButtonSendCommandFreeze, SIGNAL(clicked()), this, SLOT(onSendCommandFreeze()));
  connect(ui_->pushButtonResetCommand, SIGNAL(clicked()), this, SLOT(onResetCommand()));
  connect(ui_->comboBoxState, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(onStateChanged(const QString&)));
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void AnydriveInterface::setName(QString name) {
  ui_->labelName->setText(name);
}

void AnydriveInterface::updateAnydriveWidget(anydrive::fsm::StateEnum state, anydrive::mode::ModeEnum mode,
                                             const std::vector<std::string>& infos, const std::vector<std::string>& warnings,
                                             const std::vector<std::string>& errors, const std::vector<std::string>& fatals,
                                             double jointPosition, double jointVelocity, double jointTorque, double current, double voltage,
                                             double temperature) {
  // Set state.
  const QString stateQString = QString::fromStdString(anydrive::fsm::stateEnumToName(state));
  ui_->labelState->setText(stateQString);
  int index = ui_->comboBoxState->findText(stateQString);
  if (index != -1) {
    ui_->comboBoxState->blockSignals(true);
    ui_->comboBoxState->setCurrentIndex(index);
    ui_->comboBoxState->blockSignals(false);
  }
  // Set mode.
  ui_->labelMode->setText(QString::fromStdString(anydrive::mode::modeEnumToName(mode)));
  // Set error tooltip.
  std::string messagesTooltip;
  for (const auto& fatal : fatals) {
    messagesTooltip += fatal + "\n";
  }
  for (const auto& error : errors) {
    messagesTooltip += error + "\n";
  }
  for (const auto& warning : warnings) {
    messagesTooltip += warning + "\n";
  }
  for (const auto& info : infos) {
    messagesTooltip += info + "\n";
  }
  if (messagesTooltip.empty()) {
    messagesTooltip += "Ok.\n";
  }
  assert(!messagesTooltip.empty());
  assert(messagesTooltip[messagesTooltip.length() - 1] == '\n');
  messagesTooltip.erase(messagesTooltip.length() - 1);  // Erase last \n
  // Set error color.
  ui_->labelStatus->setToolTip(QString::fromStdString(messagesTooltip));
  if (!fatals.empty() || state == anydrive::fsm::StateEnum::Fatal) {
    ui_->labelStatus->setStyleSheet("QLabel { background-color : rgb(128, 0, 255); }");
  } else if (!errors.empty() || state == anydrive::fsm::StateEnum::Error) {
    ui_->labelStatus->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); }");
  } else if (!warnings.empty()) {
    ui_->labelStatus->setStyleSheet("QLabel { background-color : rgb(255, 128, 0); }");
  } else {
    ui_->labelStatus->setStyleSheet("QLabel { background-color : rgb(0, 255, 0); }");
  }
  // Set measurements.
  ui_->labelPosition->setText(QString::number(jointPosition, 'f', 5));
  ui_->labelVelocity->setText(QString::number(jointVelocity, 'f', 5));
  ui_->labelTorque->setText(QString::number(jointTorque, 'f', 5));
  ui_->labelCurrent->setText(QString::number(current, 'f', 5));
  ui_->labelVoltage->setText(QString::number(voltage, 'f', 1));
  ui_->labelTemperature->setText(QString::number(temperature, 'f', 1));
  setEnableCommandWidgets(state);
}

QString AnydriveInterface::getControlword() {
  return ui_->comboBoxControlword->currentText();
}

void AnydriveInterface::getCommand(anydrive::mode::ModeEnum& mode, double& position, double& velocity, double& jointTorque, double& current,
                                   double& pidGainsP, double& pidGainsI, double& pidGainsD) {
  mode = anydrive::mode::modeNameToEnum(ui_->comboBoxMode->currentText().toStdString());
  position = ui_->doubleSpinBoxPosition->value();
  velocity = ui_->doubleSpinBoxVelocity->value();
  jointTorque = ui_->doubleSpinBoxTorque->value();
  current = ui_->doubleSpinBoxCurrent->value();
  pidGainsP = ui_->doubleSpinBoxParamA->value();
  pidGainsI = ui_->doubleSpinBoxParamB->value();
  pidGainsD = ui_->doubleSpinBoxParamC->value();
}

void AnydriveInterface::setCommand(double position, double velocity, double jointTorque, double current, double pidGainsP, double pidGainsI,
                                   double pidGainsD) {
  ui_->doubleSpinBoxPosition->setValue(position);
  ui_->doubleSpinBoxVelocity->setValue(velocity);
  ui_->doubleSpinBoxTorque->setValue(jointTorque);
  ui_->doubleSpinBoxCurrent->setValue(current);
  ui_->doubleSpinBoxParamA->setValue(pidGainsP);
  ui_->doubleSpinBoxParamB->setValue(pidGainsI);
  ui_->doubleSpinBoxParamC->setValue(pidGainsD);
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

void AnydriveInterface::setEnableCommandWidgets(const anydrive::fsm::StateEnum stateEnum) {
  const bool inControlOp = (stateEnum == anydrive::fsm::StateEnum::ControlOp);

  ui_->comboBoxMode->setEnabled(inControlOp);
  ui_->doubleSpinBoxPosition->setEnabled(inControlOp);
  ui_->doubleSpinBoxVelocity->setEnabled(inControlOp);
  ui_->doubleSpinBoxTorque->setEnabled(inControlOp);
  ui_->doubleSpinBoxCurrent->setEnabled(inControlOp);
  ui_->doubleSpinBoxParamA->setEnabled(inControlOp);
  ui_->doubleSpinBoxParamB->setEnabled(inControlOp);
  ui_->doubleSpinBoxParamC->setEnabled(inControlOp);
  ui_->pushButtonResetCommand->setEnabled(inControlOp);
  ui_->pushButtonSendCommand->setEnabled(inControlOp);
  ui_->pushButtonSendCommandDisable->setEnabled(inControlOp);
  ui_->pushButtonSendCommandFreeze->setEnabled(inControlOp);
}

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void AnydriveInterface::onSendControlword() {
  emit sigSendControlword();
}

void AnydriveInterface::onResetCommand() {
  emit sigResetCommand();
}

void AnydriveInterface::onSendCommand() {
  emit sigSendCommand();
}

void AnydriveInterface::onSendCommandDisable() {
  emit sigSendCommandDisable();
}

void AnydriveInterface::onSendCommandFreeze() {
  emit sigSendCommandFreeze();
}

void AnydriveInterface::onStateChanged(const QString& state) {
  emit sigStateChanged(state);
}

}  // namespace anydrive_monitor
