/*
 * ResizableWidget.h
 *
 *  Created on: May 24, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once


// Qt
#include <QWidget>
#include <QEvent>

namespace rqt_utils {

class ResizableWidget: public QWidget
{
	Q_OBJECT
 public:
		ResizableWidget(QWidget* parent = nullptr) : QWidget(parent) {

		}

	signals:
		void widgetSizeChanged();

	protected:
		void resizeEvent(QResizeEvent * event) {
			emit widgetSizeChanged();
		}
};

} /* namespace rqt_utils */
