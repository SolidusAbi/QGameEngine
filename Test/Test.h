#pragma once

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QDockWidget>
#include <AEOpenGLHandler.h>
#include <DataLIDAR.h>
#include <AnomalyGLWindow.h>

#include "ui_Test.h"

class Test : public QMainWindow {
	Q_OBJECT

public:
	Test(QWidget *parent = Q_NULLPTR);

	void laPrueba();
	void laPrueba2();
	void keyPressEvent(QKeyEvent *event);
	void print();
	void print2();

private:
	int prueba = 0;
	Ui::TestClass ui;
	AEOpenGLHandler *handler;
	DataLIDAR *data = new DataLIDAR();
	AnomalyGLWindow *w;
};
