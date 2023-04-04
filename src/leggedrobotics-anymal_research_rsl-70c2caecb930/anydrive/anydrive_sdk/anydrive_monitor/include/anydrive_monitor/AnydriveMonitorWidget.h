#pragma once

#include <QWidget>
#include "AnydriveInterface.h"

namespace Ui {  // NOLINT
class AnydriveMonitorWidget;
}

namespace anydrive_monitor {

class AnydriveMonitorWidget : public QWidget {
  Q_OBJECT

 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit AnydriveMonitorWidget(QWidget* parent = nullptr);

  ~AnydriveMonitorWidget() override;

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  void pushBackAnydriveInterface(AnydriveInterface* anydriveInterface);

  void addSpacer();

  bool getFeedbackSectionState();

  void setFeedbackSectionState(bool state);

  bool getParametersSectionState();

  void setParametersSectionState(bool state);

  bool getCommandSectionState();

  void setCommandSectionState(bool state);

 private:
  /* ======================================================================== */
  /* Constants                                                                */
  /* ======================================================================== */

  const int widthName_ = 70;
  const int widthMinState_ = 80;
  const int widthControlword_ = 100;
  const int widthMinMode_ = 160;
  const int widthStatus_ = 50;
  const int widthPosition_ = 110;
  const int widthVelocity_ = 110;
  const int widthTorque_ = 110;
  const int widthCurrent_ = 110;
  const int widthVoltage_ = 80;
  const int widthTemperature_ = 80;
  const int widthParam_ = 100;
  const int widthCommand1_ = 100;
  const int widthCommand2_ = 100;
  const int widthExpand_ = 18;

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::AnydriveMonitorWidget* ui_;

  std::vector<AnydriveInterface*> anydriveWidgets_;

  bool isFeedbackVisible_ = true;
  bool isParametersVisible_ = false;
  bool isCommandVisible_ = true;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  void setFixedWidth();

  void setMinWidth();

  void updateFeedbackVisibility();

  void updateParametersVisibility();

  void updateCommandVisibility();

 protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  void onCheckBoxFeedbackStateChanged(int state);

  void onCheckBoxParametersStateChanged(int state);

  void onCheckBoxCommandStateChanged(int state);
};

}  // namespace anydrive_monitor
