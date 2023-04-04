#include "anydrive_monitor/AnydriveTitleWidget.h"
#include "anydrive_monitor/ui_AnydriveTitleWidget.h"

namespace anydrive_monitor {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

AnydriveTitleWidget::AnydriveTitleWidget(QWidget* parent) : QWidget(parent), ui_(new Ui::AnydriveTitleWidget) {
  ui_->setupUi(this);
}

AnydriveTitleWidget::~AnydriveTitleWidget() {
  delete ui_;
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

Ui::AnydriveTitleWidget* AnydriveTitleWidget::ui() {
  return ui_;
}

void AnydriveTitleWidget::init() {
  ui_->checkBoxFeedback->setToolTip("Show/hide feedback");
  ui_->checkBoxParameters->setToolTip("Show/hide parameters");
  ui_->checkBoxCommand->setToolTip("Show/hide command");
}

void AnydriveTitleWidget::hideFeedback() {
  ui_->labelPosition->hide();
  ui_->labelVelocity->hide();
  ui_->labelTorque->hide();
  ui_->labelCurrent->hide();
  ui_->labelVoltage->hide();
  ui_->labelTemperature->hide();
}

void AnydriveTitleWidget::showFeedback() {
  ui_->labelPosition->show();
  ui_->labelVelocity->show();
  ui_->labelTorque->show();
  ui_->labelCurrent->show();
  ui_->labelVoltage->show();
  ui_->labelTemperature->show();
}

void AnydriveTitleWidget::hideParameters() {
  ui_->labelParamA->hide();
  ui_->labelParamB->hide();
  ui_->labelParamC->hide();
}

void AnydriveTitleWidget::showParameters() {
  ui_->labelParamA->show();
  ui_->labelParamB->show();
  ui_->labelParamC->show();
}

void AnydriveTitleWidget::hideCommand() {
  ui_->labelCommand1->hide();
  ui_->labelCommand2->hide();
}

void AnydriveTitleWidget::showCommand() {
  ui_->labelCommand1->show();
  ui_->labelCommand2->show();
}

}  // namespace anydrive_monitor
