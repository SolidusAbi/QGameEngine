#include <QDebug>

#include "AEOpenGLHandler.h"
#include "AnomalyGLWindow.h"
#include "DataLIDAR.h"


AEOpenGLHandler::AEOpenGLHandler() : view_type(AEOpenGLHandlerView::AnomalyWindow), state(AEOpenGLHandlerState::Created) {
	initialize(800, 600);
}

AEOpenGLHandler::AEOpenGLHandler(AEOpenGLHandlerView view, int width, int height) : view_type(view), state(AEOpenGLHandlerState::Created) {
	initialize(width, height);
}

AEOpenGLHandler::~AEOpenGLHandler() {
	delete viewer;
	delete data;
}

void AEOpenGLHandler::initialize(int width, int height) {
	switch (view_type) {
	case AnomalyWindow:
	default:
		viewer = new AnomalyGLWindow(width, height);
		data = new DataLIDAR();
		data_type = AEOpenGLHandlerData::LIDAR;
		break;
	}

	state = AEOpenGLHandlerState::Created;
}

int AEOpenGLHandler::importDataFromFile(QString filename, QVector3D color) {
	int success;
	switch (data_type) {
	case LIDAR:
	default:
		if (color == QVector3D(-1, -1, -1))
			success = data->importData(filename);
		else
			success = data->importData(filename, color);

		break;
	}
	if (success != 0)
		return success;

	state = AEOpenGLHandlerState::DataImported;
	success = importVertex();

	return success;
}

int AEOpenGLHandler::importDataFromVector(std::vector<PointT> &vector) {
	int success = data->importData(vector);
	if (success != 0)
		return success;

	state = AEOpenGLHandlerState::DataImported;
	success = importVertex();
	return success;
}

int AEOpenGLHandler::importDataFromVector(std::vector<PointT>& vector, QVector3D color)
{
	return 0;
}

int AEOpenGLHandler::setProjectionLine(QVector3D ori, QVector3D dst) {
	if (view_type != AEOpenGLHandlerView::AnomalyWindow || data_type != AEOpenGLHandlerData::LIDAR)
		return -1;

	std::vector<Vertex> projection_line_vertices;
	Vertex ori_vertex = Vertex(data->getVertexCoordFromPoint(ori), QVector3D(255, 255, 255));
	Vertex dest_vertex = Vertex(data->getVertexCoordFromPoint(dst), QVector3D(255, 255, 255));

	static_cast<AnomalyGLWindow *>(viewer)->setProjectionLine(ori_vertex, dest_vertex);

	return 0;
}

int AEOpenGLHandler::importVertex() {
	if (state != AEOpenGLHandlerState::DataImported)
		return -1; //Cannot import vertex without data

	viewer->setVertices(data->getVertices(), GL_POINTS);

	state = AEOpenGLHandlerState::VertexImported;
	return 0;
}

int AEOpenGLHandler::show() {
	if (state != AEOpenGLHandlerState::VertexImported)
		return -1;

	viewer->show();
	return 0;
}

void AEOpenGLHandler::clean() {
	viewer->cleanGL();
	data->clear();

	state = AEOpenGLHandlerState::Cleansed;
}

int AEOpenGLHandler::setView(AnomalyView type) {
	if (view_type != AEOpenGLHandlerView::AnomalyWindow && data_type != AEOpenGLHandlerData::LIDAR)
		return -1;
	else {
		switch (type) {
		case AnomalyView::top:
			static_cast<AnomalyGLWindow *>(viewer)->topView();
			break;
		case AnomalyView::cross:
			static_cast<AnomalyGLWindow *>(viewer)->crossView();
			break;
		case AnomalyView::longitudinal:
			static_cast<AnomalyGLWindow *>(viewer)->longitudinalView();
			break;
		case AnomalyView::none:
		default:
			break;
		}
	}

	return 0;
}

int AEOpenGLHandler::setView(AnomalyView type, QVector3D center) {
	if (view_type != AEOpenGLHandlerView::AnomalyWindow && data_type != AEOpenGLHandlerData::LIDAR)
		return -1;
	else {
		QVector3D anomaly = data->getVertexCoordFromPoint(center);

		switch (type) {
		case AnomalyView::top:
			static_cast<AnomalyGLWindow *>(viewer)->topView(anomaly);
			break;
		case AnomalyView::cross:
			static_cast<AnomalyGLWindow *>(viewer)->crossView(anomaly);
			break;
		case AnomalyView::longitudinal:
			static_cast<AnomalyGLWindow *>(viewer)->longitudinalView(anomaly);
			break;
		case AnomalyView::none:
		default:
			break;
		}
	}

	return 0;
}

void AEOpenGLHandler::paint(std::vector<QVector3D>& point, QVector3D & color, GLenum primitive_type) {
	if (point.size() == 0)
		return;

	std::vector<Vertex> vertices;
	for (size_t point_index = 0; point_index < point.size(); ++point_index) {
		QVector3D vertex_coord = data->getVertexCoordFromPoint(point.at(point_index));
		vertices.push_back(Vertex(vertex_coord, color));
	}

	viewer->setVertices(vertices, primitive_type);
}
