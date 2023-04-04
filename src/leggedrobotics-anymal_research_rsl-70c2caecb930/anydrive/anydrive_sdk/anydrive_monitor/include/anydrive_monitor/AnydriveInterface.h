#pragma once

#include <QWidget>
#include <anydrive/fsm/StateEnum.hpp>
#include <anydrive/mode/ModeEnum.hpp>

#include "anydrive_monitor/AnydriveWidget.h"

namespace anydrive_monitor {

class AnydriveInterface : public AnydriveWidget {
  Q_OBJECT

 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit AnydriveInterface(QWidget* parent = nullptr);

  ~AnydriveInterface() override = default;

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  void setName(QString name);

  void updateAnydriveWidget(anydrive::fsm::StateEnum state, anydrive::mode::ModeEnum mode, const std::vector<std::string>& infos,
                            const std::vector<std::string>& warnings, const std::vector<std::string>& errors,
                            const std::vector<std::string>& fatals, double jointPosition, double jointVelocity, double jointTorque,
                            double current, double voltage, double temperature);

  QString getControlword();

  void getCommand(anydrive::mode::ModeEnum& mode, double& position, double& velocity, double& jointTorque, double& current,
                  double& pidGainsP, double& pidGainsI, double& pidGainsD);

  void setCommand(double position, double velocity, double jointTorque, double current, double pidGainsP, double pidGainsI,
                  double pidGainsD);

 private:
  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  void setEnableCommandWidgets(const anydrive::fsm::StateEnum stateEnum);

 protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  void onSendControlword();

  void onResetCommand();

  void onSendCommand();

  void onSendCommandDisable();

  void onSendCommandFreeze();

  void onStateChanged(const QString& state);

 signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */

  void sigSendControlword();

  void sigResetCommand();

  void sigSendCommand();

  void sigSendCommandDisable();

  void sigSendCommandFreeze();

  void sigStateChanged(const QString& state);
};

}  // namespace anydrive_monitor
