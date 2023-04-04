#pragma once

#include <QPushButton>
#include <QWidget>

namespace Ui {  // NOLINT
class AnydriveTitleWidget;
}

namespace anydrive_monitor {

class AnydriveTitleWidget : public QWidget {
  Q_OBJECT

 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit AnydriveTitleWidget(QWidget* parent = nullptr);

  ~AnydriveTitleWidget() override;

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  Ui::AnydriveTitleWidget* ui();

  void init();

  void hideFeedback();

  void showFeedback();

  void hideParameters();

  void showParameters();

  void hideCommand();

  void showCommand();

 private:
  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::AnydriveTitleWidget* ui_;
};

}  // namespace anydrive_monitor
