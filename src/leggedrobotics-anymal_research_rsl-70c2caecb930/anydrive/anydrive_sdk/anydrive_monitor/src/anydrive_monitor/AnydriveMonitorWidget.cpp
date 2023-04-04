#include "anydrive_monitor/AnydriveMonitorWidget.h"
#include "anydrive_monitor/ui_AnydriveMonitorWidget.h"
#include "anydrive_monitor/ui_AnydriveTitleWidget.h"
#include "anydrive_monitor/ui_AnydriveWidget.h"

namespace anydrive_monitor {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

AnydriveMonitorWidget::AnydriveMonitorWidget(QWidget* parent) : QWidget(parent), ui_(new Ui::AnydriveMonitorWidget) {
  ui_->setupUi(this);

  // Connect signals, slots.
  connect(ui_->widgetAnydriveTitle->ui()->checkBoxFeedback, SIGNAL(stateChanged(int)), this, SLOT(onCheckBoxFeedbackStateChanged(int)));
  connect(ui_->widgetAnydriveTitle->ui()->checkBoxParameters, SIGNAL(stateChanged(int)), this, SLOT(onCheckBoxParametersStateChanged(int)));
  connect(ui_->widgetAnydriveTitle->ui()->checkBoxCommand, SIGNAL(stateChanged(int)), this, SLOT(onCheckBoxCommandStateChanged(int)));

  // Initialize AnydriveTitleWidget.
  ui_->widgetAnydriveTitle->init();

  // Update expand/collapse.
  ui_->widgetAnydriveTitle->ui()->checkBoxFeedback->setChecked(isFeedbackVisible_);
  ui_->widgetAnydriveTitle->ui()->checkBoxParameters->setChecked(isParametersVisible_);
  ui_->widgetAnydriveTitle->ui()->checkBoxCommand->setChecked(isCommandVisible_);
  updateFeedbackVisibility();
  updateParametersVisibility();
  updateCommandVisibility();

  // Update fixed width.
  setFixedWidth();

  // Update minimum width.
  setMinWidth();
}

AnydriveMonitorWidget::~AnydriveMonitorWidget() {
  delete ui_;
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void AnydriveMonitorWidget::pushBackAnydriveInterface(AnydriveInterface* anydriveInterface) {
  anydriveWidgets_.push_back(anydriveInterface);
  ui_->verticalLayout->addWidget(anydriveWidgets_.back());

  QPalette pal;
  QColor background;
  static int counter = 0;
  if (counter % 2 != 0) {
    background.setNamedColor("#d3d3d3");
  } else {
    background.setNamedColor("#f2f2f2");
    background.setNamedColor("#fcfcfc");
  }
  pal.setColor(QPalette::Background, background);
  anydriveWidgets_.back()->setAutoFillBackground(true);
  anydriveWidgets_.back()->setPalette(pal);
  anydriveWidgets_.back()->ui()->line->hide();
  counter++;

  updateFeedbackVisibility();
  updateParametersVisibility();
  updateCommandVisibility();

  // Update fixed width.
  setFixedWidth();

  // Update minimum width.
  setMinWidth();
}

void AnydriveMonitorWidget::addSpacer() {
  ui_->verticalLayout->addItem(new QSpacerItem(5, 0, QSizePolicy::Minimum, QSizePolicy::Expanding));
}

bool AnydriveMonitorWidget::getFeedbackSectionState() {
  return ui_->widgetAnydriveTitle->ui()->checkBoxFeedback->isChecked();
}

void AnydriveMonitorWidget::setFeedbackSectionState(bool state) {
  ui_->widgetAnydriveTitle->ui()->checkBoxFeedback->setChecked(state);
  isFeedbackVisible_ = state;
  updateFeedbackVisibility();
}

bool AnydriveMonitorWidget::getParametersSectionState() {
  return ui_->widgetAnydriveTitle->ui()->checkBoxParameters->isChecked();
}

void AnydriveMonitorWidget::setParametersSectionState(bool state) {
  ui_->widgetAnydriveTitle->ui()->checkBoxParameters->setChecked(state);
  isParametersVisible_ = state;
  updateParametersVisibility();
}

bool AnydriveMonitorWidget::getCommandSectionState() {
  return ui_->widgetAnydriveTitle->ui()->checkBoxCommand->isChecked();
}

void AnydriveMonitorWidget::setCommandSectionState(bool state) {
  ui_->widgetAnydriveTitle->ui()->checkBoxCommand->setChecked(state);
  isCommandVisible_ = state;
  updateCommandVisibility();
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

void AnydriveMonitorWidget::setFixedWidth() {
  ui_->widgetAnydriveTitle->ui()->labelName->setFixedWidth(widthName_);
  ui_->widgetAnydriveTitle->ui()->labelControlword->setFixedWidth(widthControlword_);
  ui_->widgetAnydriveTitle->ui()->labelStatus->setFixedWidth(widthStatus_);
  ui_->widgetAnydriveTitle->ui()->labelPosition->setFixedWidth(widthPosition_);
  ui_->widgetAnydriveTitle->ui()->labelVelocity->setFixedWidth(widthVelocity_);
  ui_->widgetAnydriveTitle->ui()->labelTorque->setFixedWidth(widthTorque_);
  ui_->widgetAnydriveTitle->ui()->labelCurrent->setFixedWidth(widthCurrent_);
  ui_->widgetAnydriveTitle->ui()->labelVoltage->setFixedWidth(widthVoltage_);
  ui_->widgetAnydriveTitle->ui()->labelTemperature->setFixedWidth(widthTemperature_);
  ui_->widgetAnydriveTitle->ui()->labelParamA->setFixedWidth(widthParam_);
  ui_->widgetAnydriveTitle->ui()->labelParamB->setFixedWidth(widthParam_);
  ui_->widgetAnydriveTitle->ui()->labelParamC->setFixedWidth(widthParam_);
  ui_->widgetAnydriveTitle->ui()->labelCommand1->setFixedWidth(widthCommand1_);
  ui_->widgetAnydriveTitle->ui()->labelCommand2->setFixedWidth(widthCommand2_);

  ui_->widgetAnydriveTitle->ui()->checkBoxFeedback->setFixedWidth(widthExpand_);
  ui_->widgetAnydriveTitle->ui()->checkBoxParameters->setFixedWidth(widthExpand_);
  ui_->widgetAnydriveTitle->ui()->checkBoxCommand->setFixedWidth(widthExpand_);

  for (auto& item : anydriveWidgets_) {
    item->ui()->labelName->setFixedWidth(widthName_);
    item->ui()->comboBoxControlword->setFixedWidth(widthControlword_);
    item->ui()->pushButtonSendControlword->setFixedWidth(widthControlword_);
    item->ui()->labelStatus->setFixedWidth(widthStatus_);
    item->ui()->labelPosition->setFixedWidth(widthPosition_);
    item->ui()->doubleSpinBoxPosition->setFixedWidth(widthPosition_);
    item->ui()->labelVelocity->setFixedWidth(widthVelocity_);
    item->ui()->doubleSpinBoxVelocity->setFixedWidth(widthVelocity_);
    item->ui()->labelTorque->setFixedWidth(widthTorque_);
    item->ui()->doubleSpinBoxTorque->setFixedWidth(widthTorque_);
    item->ui()->labelCurrent->setFixedWidth(widthCurrent_);
    item->ui()->doubleSpinBoxCurrent->setFixedWidth(widthCurrent_);
    item->ui()->labelVoltage->setFixedWidth(widthVoltage_);
    item->ui()->labelTemperature->setFixedWidth(widthTemperature_);
    item->ui()->doubleSpinBoxParamA->setFixedWidth(widthParam_);
    item->ui()->doubleSpinBoxParamB->setFixedWidth(widthParam_);
    item->ui()->doubleSpinBoxParamC->setFixedWidth(widthParam_);
    item->ui()->pushButtonResetCommand->setFixedWidth(widthCommand1_);
    item->ui()->pushButtonSendCommand->setFixedWidth(widthCommand1_);
    item->ui()->pushButtonSendCommandDisable->setFixedWidth(widthCommand2_);
    item->ui()->pushButtonSendCommandFreeze->setFixedWidth(widthCommand2_);

    item->ui()->labelExpandFeedback1->setFixedWidth(widthExpand_);
    item->ui()->labelExpandFeedback2->setFixedWidth(widthExpand_);
    item->ui()->labelExpandParameters1->setFixedWidth(widthExpand_);
    item->ui()->labelExpandParameters2->setFixedWidth(widthExpand_);
    item->ui()->labelExpandCommand1->setFixedWidth(widthExpand_);
    item->ui()->labelExpandCommand2->setFixedWidth(widthExpand_);
  }
}

void AnydriveMonitorWidget::setMinWidth() {
  ui_->widgetAnydriveTitle->ui()->labelState->setMinimumWidth(widthMinState_);
  ui_->widgetAnydriveTitle->ui()->labelMode->setMinimumWidth(widthMinMode_);

  for (auto& item : anydriveWidgets_) {
    item->ui()->labelState->setMinimumWidth(widthMinState_);
    item->ui()->comboBoxState->setMinimumWidth(widthMinState_);
    item->ui()->labelMode->setMinimumWidth(widthMinMode_);
    item->ui()->comboBoxMode->setMinimumWidth(widthMinMode_);
  }
}

void AnydriveMonitorWidget::updateFeedbackVisibility() {
  if (isFeedbackVisible_) {
    ui_->widgetAnydriveTitle->showFeedback();
    for (auto& item : anydriveWidgets_) {
      item->showFeedback();
    }
  } else {
    ui_->widgetAnydriveTitle->hideFeedback();
    for (auto& item : anydriveWidgets_) {
      item->hideFeedback();
    }
  }
}

void AnydriveMonitorWidget::updateParametersVisibility() {
  if (isParametersVisible_) {
    ui_->widgetAnydriveTitle->showParameters();
    for (auto& item : anydriveWidgets_) {
      item->showParameters();
    }
  } else {
    ui_->widgetAnydriveTitle->hideParameters();
    for (auto& item : anydriveWidgets_) {
      item->hideParameters();
    }
  }
}

void AnydriveMonitorWidget::updateCommandVisibility() {
  if (isCommandVisible_) {
    ui_->widgetAnydriveTitle->showCommand();
    for (auto& item : anydriveWidgets_) {
      item->showCommand();
    }
  } else {
    ui_->widgetAnydriveTitle->hideCommand();
    for (auto& item : anydriveWidgets_) {
      item->hideCommand();
    }
  }
}

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void AnydriveMonitorWidget::onCheckBoxFeedbackStateChanged(int state) {
  isFeedbackVisible_ = static_cast<bool>(state);
  updateFeedbackVisibility();
}

void AnydriveMonitorWidget::onCheckBoxParametersStateChanged(int state) {
  isParametersVisible_ = static_cast<bool>(state);
  updateParametersVisibility();
}

void AnydriveMonitorWidget::onCheckBoxCommandStateChanged(int state) {
  isCommandVisible_ = static_cast<bool>(state);
  updateCommandVisibility();
}

}  // namespace anydrive_monitor
