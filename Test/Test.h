#pragma once

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QDockWidget>
#include <AEOpenGLHandler.h>

#include "ui_Test.h"

class Test : public QMainWindow {
	Q_OBJECT

public:
	Test(QWidget *parent = Q_NULLPTR);

	void laPrueba();

private:
	Ui::TestClass ui;
	QDockWidget *dock;
	AEOpenGLHandler *handler;
};
