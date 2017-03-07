#include "Test.h"

Test::Test(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);
	handler = new AEOpenGLHandler(AEOpenGLHandlerView::AnomalyWindow, 800, 600);
	//handler->getViewer()->setMouseTracking(true);
	setCentralWidget(handler->getViewer());
}

void Test::laPrueba() {
	std::vector<QVector3D> projection_line;
	projection_line.push_back(QVector3D(-55290.3, 25429.6, 1000));
	projection_line.push_back(QVector3D(-83117.1, 45855.9, 1000));

	handler->importDataFromFile("C:\\Users\\ahguedes\\Desktop\\Datos\\1_1.pcd", QVector3D(0, 255, 0));
	//handler->importDataFromFile("C:\\Users\\ahguedes\\Desktop\\Datos\\cables.pcd", QVector3D(0, 255, 255));
	handler->setProjectionLine(projection_line.at(0), projection_line.at(1));
	handler->setView(AnomalyView::longitudinal, QVector3D(-70064.4, 35863.5, 1214));
	std::vector<QVector3D> estomismo;
	estomismo.push_back(QVector3D(-70064.4, 35863.5, 1214));
	estomismo.push_back(QVector3D(-70061.8, 35876.9, 1847.2));
	handler->paint(estomismo, QVector3D(255, 255, 255), GL_LINES);
	handler->show();
	handler->setProjectionDepth(40 * 100);
	handler->setProjectionWidth(50 * 100);
}
