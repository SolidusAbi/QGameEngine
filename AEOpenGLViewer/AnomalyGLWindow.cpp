#include <math.h>
#include <limits.h>

#define _USE_MATH_DEFINES

#include "Input.h"
#include "AnomalyGLWindow.h"

AnomalyGLWindow::AnomalyGLWindow() :AEOpenGLViewer(GLWindowViewConfig::Orthogonal, GLWindowMoveConfig::Freedom),
anomaly_view(AnomalyView::none), anomaly_point(QVector3D(0, 0, 0)) {}

AnomalyGLWindow::AnomalyGLWindow(int width, int height) : AEOpenGLViewer(GLWindowViewConfig::Orthogonal, GLWindowMoveConfig::Freedom, width, height),
anomaly_view(AnomalyView::none), anomaly_point(QVector3D(0, 0, 0)) {}

AnomalyGLWindow::AnomalyGLWindow(Vertex line_coord_0, Vertex line_coord_1) : AEOpenGLViewer(GLWindowViewConfig::Orthogonal, GLWindowMoveConfig::Freedom),
anomaly_view(AnomalyView::none), anomaly_point(QVector3D(0, 0, 0)) {
	setProjectionLine(line_coord_0, line_coord_1);
}

AnomalyGLWindow::AnomalyGLWindow(Vertex line_coord_0, Vertex line_coord_1, int window_width, int window_height) :
	AEOpenGLViewer(GLWindowViewConfig::Orthogonal, GLWindowMoveConfig::Freedom, window_width, window_height) {
	setProjectionLine(line_coord_0, line_coord_1);
}

AnomalyGLWindow::~AnomalyGLWindow() {}

void AnomalyGLWindow::setAnomalyPoint(QVector3D anomaly_point) {
	this->anomaly_point = anomaly_point;
	reset(); // Centering camera in the anomaly point
}

void AnomalyGLWindow::setProjectionLine(Vertex coord_0, Vertex coord_1) {
	if (projection_line_vertices.size() > 0)
		projection_line_vertices.clear();

	projection_line_vertices.push_back(coord_0);
	projection_line_vertices.push_back(coord_1);

	if (coord_1.getPosition().x() != coord_0.getPosition().x()) {
		line_slope = (coord_1.getPosition().y() - coord_0.getPosition().y()) / (coord_1.getPosition().x() - coord_0.getPosition().x());
		line_angle = atan(line_slope) * 180 / M_PI;
	}
	else {
		line_slope = std::numeric_limits<float>::max(); //Inf
		line_angle = 90;
	}

	//To draw line vertices  (Just for test)
	//setVertices(std::vector<Vertex>{coord_0, coord_1}, GL_LINES);
}

int AnomalyGLWindow::topView(QVector3D anomaly_point) {
	setAnomalyPoint(anomaly_point);
	return topView();
}

int AnomalyGLWindow::topView() {
	if (!lineDataPrepare())
		return -1;

	if (anomaly_view != AnomalyView::top)
		reset();

	//We will modify camera matrix, translate it to the anomaly point (anomaly point will be located in (0,0) coord)
	anomaly_view = AnomalyView::top;

	return 0;
}

int AnomalyGLWindow::crossView(QVector3D anomaly_point) {
	setAnomalyPoint(anomaly_point);
	return crossView();
}

int AnomalyGLWindow::crossView() {
	if (!lineDataPrepare())
		return -1;

	if (anomaly_view != AnomalyView::cross)
		reset();

	anomaly_view = AnomalyView::cross;

	camera.rotate(90, Camera3D::LocalRight);
	camera.rotate(-(line_angle + 90), Camera3D::LocalForward);

	update();

	return 0;
}

int AnomalyGLWindow::longitudinalView(QVector3D anomaly_point) {
	setAnomalyPoint(anomaly_point);
	return longitudinalView();
}

int AnomalyGLWindow::longitudinalView() {
	if (!lineDataPrepare())
		return -1;

	if (anomaly_view != AnomalyView::longitudinal)
		reset();

	anomaly_view = AnomalyView::longitudinal;

	camera.rotate(90, Camera3D::LocalRight);
	camera.rotate(-(line_angle + 180), Camera3D::LocalForward);

	return 0;
}

void AnomalyGLWindow::reset() {
	camera.restore();

	//Center the view in the anomaly
	camera.translate(camera.right(anomaly_point.x()));
	camera.translate(camera.up(anomaly_point.y()));
	camera.translate(-(camera.forward(anomaly_point.z())));

	switch (anomaly_view) {
	case AnomalyGLWindow::top:
		topView();
		break;
	case AnomalyGLWindow::cross:
		crossView();
		break;
	case AnomalyGLWindow::longitudinal:
		longitudinalView();
		break;
	default:
		AnomalyView::none;
		break;
	}
}

void AnomalyGLWindow::inputHandle(int key) {
	// Camera Transformation
	if (button_pressed) {
		static const float transSpeed = 1.0f;

		// Handle translations
		QVector3D translation;
		switch (key) {
		case Qt::Key_W:
			translation += camera.up();
			break;
		case Qt::Key_S:
			translation -= camera.up();
			break;
		case Qt::Key_A:
			translation -= camera.right();
			break;
		case Qt::Key_D:
			translation += camera.right();
			break;
		case Qt::Key_R:
			reset();
			break;
		case Qt::Key_C:
			cleanGL();
			break;
		case Qt::Key_I:
			toImage("C:/Users/ahguedes/Desktop/Prueba/prueba3.png");
			break;
		default:
			break;
		}
		camera.translate(transSpeed * translation);
	}
}

void AnomalyGLWindow::mouseMoveEvent(QMouseEvent * event) {
	float ratio = width() / float(height());
	QRect viewport(0, 0, this->width(), this->height());

	QPoint mouseCurrPosition = event->pos();
	QVector3D cursor_point = QVector3D(mouseCurrPosition.x(), height() - mouseCurrPosition.y(), 1);
	QVector3D cursor_prev_point = QVector3D(mousePrevPosition.x(), height() - mousePrevPosition.y(), 1);

	QVector3D cursor_delta = cursor_point - cursor_prev_point;

	if (button_pressed) {
		float steps_x = cursor_delta.x() * ortho_width / width();
		float steps_y = cursor_delta.y() * ortho_width / height();

		// Movement based in setOrthogonalProjection method.
		if (width() <= height()) {
			camera.translate(-camera.right(steps_x));
			camera.translate(-camera.up(steps_y / ratio));
		}
		else {
			camera.translate(-camera.right(steps_x * ratio));
			camera.translate(-camera.up(steps_y));
		}
	}

	update();
	mousePrevPosition = mouseCurrPosition;
}

void AnomalyGLWindow::wheelEvent(QWheelEvent *event) {
	static const float speed = 1.1f;

	if (view_configuration == GLWindowViewConfig::Orthogonal) {
		switch (event->modifiers()) {
		case Qt::KeyboardModifier::NoModifier:
			if (event->delta() > 0)
				ortho_width /= speed;
			else
				ortho_width *= speed;
			break;

		case Qt::KeyboardModifier::ShiftModifier:
			if (event->delta() > 0)
				ortho_clipping /= speed;
			else
				ortho_clipping *= speed;
			break;

		default:
			break;
		}

		setOrthogonalProjection(width(), height());
		update();
	}
}