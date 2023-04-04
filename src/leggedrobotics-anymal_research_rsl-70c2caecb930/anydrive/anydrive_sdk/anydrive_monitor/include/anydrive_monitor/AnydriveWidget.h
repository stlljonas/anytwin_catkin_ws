#pragma once

#include <QWidget>

namespace Ui {  // NOLINT
class AnydriveWidget;
}

namespace anydrive_monitor {

class AnydriveWidget : public QWidget {
  Q_OBJECT

 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit AnydriveWidget(QWidget* parent = nullptr);

  ~AnydriveWidget() override;

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  Ui::AnydriveWidget* ui();

  void reset();

  void hideFeedback();

  void showFeedback();

  void hideParameters();

  void showParameters();

  void hideCommand();

  void showCommand();

 protected:
  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::AnydriveWidget* ui_;

 protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  void onNameUpdate(QString name);

  void onStateUpdate(QString state);

  void onModeUpdate(QString mode);

  void onPositionUpdate(double position);

  void onVelocityUpdate(double velocity);

  void onTorqueUpdate(double torque);

  void onCurrentUpdate(double current);

  void onVoltageUpdate(double voltage);

  void onTemperatureUpdate(double temperature);
};

}  // namespace anydrive_monitor
