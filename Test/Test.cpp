#include <QKeyEvent>

#include "Test.h"

Test::Test(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);
	//handler = new AEOpenGLHandler(AEOpenGLHandlerView::AnomalyWindow, 800, 600);
	//handler->getViewer()->setFocusPolicy(Qt::ClickFocus);

	data = new DataLIDAR();
	w = new AnomalyGLWindow(800, 600);

	//setCentralWidget(handler->getViewer());
	setCentralWidget(w);
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

//	handler->getViewer()->toImage("C:\\Users\\ahguedes\\Desktop\\Datos\\prueba.png");
}

void Test::laPrueba2() {
	std::vector<QVector3D> projection_line;
	projection_line.push_back(QVector3D(-55290.3, 25429.6, 1000));
	projection_line.push_back(QVector3D(-83117.1, 45855.9, 1000));

 	data->importData("C:\\Users\\ahguedes\\Desktop\\Datos\\1_1.pcd", QVector3D(0, 255, 0));
 	std::vector<Vertex> prueba = data->getVertices();
 	w->setVertices(prueba, GL_POINTS);
 	w->setProjectionLine(projection_line.at(0), projection_line.at(1));
 	//w->longitudinalView(QVector3D(-70064.4, 35863.5, 1214));
	w->longitudinalView();
	
 	std::vector<Vertex> estomismo;
 	estomismo.push_back(Vertex(QVector3D(-70064.4, 35863.5, 1214), QVector3D(255, 255, 255)));
 	estomismo.push_back(Vertex(QVector3D(-70061.8, 35876.9, 1847.2) , QVector3D(255, 255, 255)));
 	w->setVertices(estomismo, GL_LINES);
	w->show();
	w->setProjectionWidth(50 * 100);
	w->setProjectionClippingDepth(40 * 100);

	estomismo.clear();
	estomismo.push_back(Vertex(QVector3D(0, 0, 0), QVector3D(255, 255, 255)));
	estomismo.push_back(Vertex(QVector3D(1000, 100, 10), QVector3D(255, 255, 255)));
	w->setVertices(estomismo, GL_LINES);

	//w->toImage("C:\\Users\\ahguedes\\Desktop\\Datos\\prueba.png");
}

void Test::keyPressEvent(QKeyEvent *event) {
	if (event->key() == Qt::Key_I) {
		//handler->importDataFromFile("C:\\Users\\ahguedes\\Desktop\\Datos\\cables.pcd", QVector3D(0, 255, 255));
 		/*DataLIDAR data2;
 		data2.importData("C:\\Users\\ahguedes\\Desktop\\Datos\\cables.pcd", QVector3D(0, 255, 255));
 		std::vector<Vertex> prueba = data2.getVertices();
 		w->setVertices(prueba, GL_POINTS);*/

// 		std::vector<Vertex> estomismo;
// 		estomismo.push_back(Vertex(QVector3D(-70064.4, 35863.5, 1214), QVector3D(255, 255, 255)));
// 		estomismo.push_back(Vertex(QVector3D(-70061.8, 35876.9, 1847.2) , QVector3D(255, 255, 255)));
// 		w->setVertices(estomismo, GL_LINES);
		//print();
		print2();
	}
	if (event->key() == Qt::Key_P) {
		prueba += 100;

		std::vector<Vertex> estomismo;
		estomismo.push_back(Vertex(QVector3D(0, 0+prueba, 0), QVector3D(255, 255, 255)));
		estomismo.push_back(Vertex(QVector3D(1000, 100+prueba, 0), QVector3D(255, 255, 255)));
		w->setVertices(estomismo, GL_LINES);
	}	
	if (event->key() == Qt::Key_O)
		w->cleanGL();
}

void Test::print() {
	qDebug() << handler->getViewer()->toImage("C:\\Users\\ahguedes\\Desktop\\Datos\\prueba.png");
}

void Test::print2() {
	qDebug() << w->toImage("C:\\Users\\ahguedes\\Desktop\\Datos\\prueba.png");
}
