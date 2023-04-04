#pragma once

#include <QWidget>

#include <environment_item_qt_ros/EnvironmentItemWidgetInterfaceRos.hpp>

#include "inspection_example/Item.hpp"

namespace Ui {  // NOLINT
class InspectionExampleWidget;
}

namespace inspection_example_qt_plugin {

class InspectionExampleWidget : public environment_item_qt_ros::EnvironmentItemWidgetInterfaceRos {
  Q_OBJECT

 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit InspectionExampleWidget(QWidget* parent = nullptr);

  ~InspectionExampleWidget() override;

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  bool eventFilter(QObject* object, QEvent* event) override;

 private:
  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::InspectionExampleWidget* ui_;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  void constructImpl() override;

  void setItemImpl(environment_item::ItemBasePtr item) override;

  environment_item::ItemBasePtr getItemImpl() override;

  void setChildrenListImpl(const QMap<int, QString>& children, const QVector<int>& relations) override;

  QVector<int> getChildrenRelationIdsImpl() override;

  void resetImpl() override;

  void setToDefaultImpl() override;
};

}  // namespace inspection_example_qt_plugin
