#include <math.h>
#include <limits.h>

#define _USE_MATH_DEFINES

#include "Input.h"
#include "AEOrthoViewer.h"

AEOrthoViewer::AEOrthoViewer() :AEOpenGLViewer(GLWindowViewConfig::Orthogonal, GLWindowMoveConfig::Freedom),
anomaly_view(AnomalyView::none), anomaly_point(QVector3D(0, 0, 0)) {}

AEOrthoViewer::AEOrthoViewer(int width, int height) : AEOpenGLViewer(GLWindowViewConfig::Orthogonal, GLWindowMoveConfig::Freedom, width, height),
anomaly_view(AnomalyView::none), anomaly_point(QVector3D(0, 0, 0)) {}

AEOrthoViewer::AEOrthoViewer(Vertex line_coord_0, Vertex line_coord_1) : AEOpenGLViewer(GLWindowViewConfig::Orthogonal, GLWindowMoveConfig::Freedom),
anomaly_view(AnomalyView::none), anomaly_point(QVector3D(0, 0, 0)) {
	setProjectionLine(line_coord_0, line_coord_1);
}

AEOrthoViewer::AEOrthoViewer(Vertex line_coord_0, Vertex line_coord_1, int window_width, int window_height) :
	AEOpenGLViewer(GLWindowViewConfig::Orthogonal, GLWindowMoveConfig::Freedom, window_width, window_height) {
	setProjectionLine(line_coord_0, line_coord_1);
}

AEOrthoViewer::~AEOrthoViewer() {}

void AEOrthoViewer::setAnomalyPoint(QVector3D anomaly_point) {
	this->anomaly_point = anomaly_point;
	reset(); // Centering camera in the anomaly point
}

void AEOrthoViewer::setProjectionLine(Vertex coord_0, Vertex coord_1) {
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

int AEOrthoViewer::topView(QVector3D anomaly_point) {
	setAnomalyPoint(anomaly_point);
	return topView();
}

int AEOrthoViewer::topView() {
	if (!lineDataPrepare())
		return -1;

	if (anomaly_view != AnomalyView::top)
		reset();

	//We will modify camera matrix, translate it to the anomaly point (anomaly point will be located in (0,0) coord)
	anomaly_view = AnomalyView::top;

	return 0;
}

int AEOrthoViewer::crossView(QVector3D anomaly_point) {
	setAnomalyPoint(anomaly_point);
	return crossView();
}

int AEOrthoViewer::crossView() {
	if (!lineDataPrepare())
		return -1;

	if (anomaly_view != AnomalyView::cross) {
		anomaly_view = AnomalyView::cross;
		reset(AnomalyView::cross);
	} else
		settingCrossViewCamera();

	/*anomaly_view = AnomalyView::cross;

	camera.rotate(90, Camera3D::LocalRight);
	camera.rotate(-(line_angle + 90), Camera3D::LocalForward);*/

	return 0;
}

int AEOrthoViewer::longitudinalView(QVector3D anomaly_point) {
	setAnomalyPoint(anomaly_point);
	return longitudinalView();
}

int AEOrthoViewer::longitudinalView() {
	if (!lineDataPrepare())
		return -1;
	
	if (anomaly_view != AnomalyView::longitudinal) {
		anomaly_view = AnomalyView::longitudinal;
		reset(anomaly_view);
	} else {
		settingLongitudinalViewCamera();
	}

	return 0;
}

void AEOrthoViewer::cleanGL(){
	anomaly_view = AnomalyView::none;
	AEOpenGLViewer::cleanGL();
}

void AEOrthoViewer::reset(AnomalyView view) {
	camera.restore();

	//Center the view in the anomaly
	camera.translate(camera.right(anomaly_point.x()));
	camera.translate(camera.up(anomaly_point.y()));
	camera.translate(-(camera.forward(anomaly_point.z())));

	switch (view) {
	    case AnomalyView::top:
		    topView();
			break;
		case AnomalyView::cross:
			settingCrossViewCamera();
			break;
		case AnomalyView::longitudinal:
			settingLongitudinalViewCamera();
			break;
		default:
			break;
	}
}

void AEOrthoViewer::inputHandle(int key) {
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
			reset(anomaly_view);
			break;
		case Qt::Key_C:
			cleanGL();
			break;
		default:
			break;
		}
		camera.translate(transSpeed * translation);
	}
}

void AEOrthoViewer::mouseMoveEvent(QMouseEvent * event) {
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

void AEOrthoViewer::wheelEvent(QWheelEvent *event) {
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

void AEOrthoViewer::settingLongitudinalViewCamera() {
	camera.rotate(90, Camera3D::LocalRight);
	camera.rotate(-(line_angle + 180), Camera3D::LocalForward);
}

void AEOrthoViewer::settingCrossViewCamera() {
	camera.rotate(90, Camera3D::LocalRight);
	camera.rotate(-(line_angle + 90), Camera3D::LocalForward);
}