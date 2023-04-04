#include <inspection_example/Item.hpp>

#include "inspection_example_qt_plugin/InspectionExampleWidget.hpp"
#include "inspection_example_qt_plugin/ui_InspectionExampleWidget.h"

namespace inspection_example_qt_plugin {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

InspectionExampleWidget::InspectionExampleWidget(QWidget* parent)
    : environment_item_qt_ros::EnvironmentItemWidgetInterfaceRos(parent), ui_(new Ui::InspectionExampleWidget) {
  ui_->setupUi(this);

  if (QString(inspection_example::Item::childCategory_).isEmpty()) {
    ui_->widgetRelation->hide();
    ui_->labelRelation->hide();
  }

  // Event filter.
  ui_->spinBoxCountTo->installEventFilter(this);

  // Connect.
}

InspectionExampleWidget::~InspectionExampleWidget() {
  delete ui_;
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

bool InspectionExampleWidget::eventFilter(QObject* object, QEvent* event) {
  if (event->type() == QEvent::Wheel) {
    return true;
  }
  return QObject::eventFilter(object, event);
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

void InspectionExampleWidget::constructImpl() {
  // Do some stuff to construct/initialize.
}

void InspectionExampleWidget::setItemImpl(environment_item::ItemBasePtr item) {
  if (item == nullptr) {
    setToDefault();
    return;
  }
  std::shared_ptr<inspection_example::Item> itemC = std::dynamic_pointer_cast<inspection_example::Item>(item);

  ui_->widgetDescription->setName(itemC->name_);
  ui_->widgetDescription->setLabel(itemC->label_);
  ui_->spinBoxCountTo->setValue(static_cast<int>(itemC->countTo_));
  ui_->lineEditFrameId->setText(itemC->frameId_.c_str());
  ui_->widgetPose->setPose(itemC->pose_);
}

environment_item::ItemBasePtr InspectionExampleWidget::getItemImpl() {
  std::shared_ptr<inspection_example::Item> item = std::make_shared<inspection_example::Item>();

  item->name_ = ui_->widgetDescription->getName();
  item->label_ = ui_->widgetDescription->getLabel();
  item->countTo_ = ui_->spinBoxCountTo->value();
  item->frameId_ = ui_->lineEditFrameId->text().toStdString();
  item->pose_ = ui_->widgetPose->getPose();

  return item;
}

void InspectionExampleWidget::setChildrenListImpl(const QMap<int, QString>& children, const QVector<int>& relations) {
  ui_->widgetRelation->setData(children, relations);
}

QVector<int> InspectionExampleWidget::getChildrenRelationIdsImpl() {
  return ui_->widgetRelation->getConnectedIds();
}

void InspectionExampleWidget::resetImpl() {
  environment_item::ItemBasePtr item = std::make_shared<inspection_example::Item>();
  setItemImpl(item);
}

void InspectionExampleWidget::setToDefaultImpl() {
  ui_->widgetDescription->setName("");
  ui_->widgetDescription->setLabel("");
  ui_->spinBoxCountTo->setValue(10);
  ui_->lineEditFrameId->setText("map");
  ui_->widgetPose->setPose({});
}

}  // namespace inspection_example_qt_plugin
