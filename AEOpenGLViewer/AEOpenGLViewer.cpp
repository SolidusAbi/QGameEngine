#include <QDebug>
#include <QString>
#include <QKeyEvent>
#include <QOpenGLFramebufferObjectFormat>
#include <QFileInfo>

#include "AEOpenGLViewer.h"
#include "vertex.h"
#include "Input.h"

//Vertex shader
const std::string vertexShader = {
	"#version 450\n"
	"layout(location = 0)in vec3 position;\n"
	"layout(location = 1) in vec3 color;\n"
	"out vec4 vColor;\n"
	"uniform mat4 modelToWorld;\n"
	"uniform mat4 worldToCamera;\n"
	"uniform mat4 cameraToView;\n"
	"void main(){"
	"gl_Position = cameraToView * worldToCamera * modelToWorld * vec4(position, 1.0);\n"
	"vColor = vec4(color, 1.0);\n"
	"}"
};

//Fragment shader
const std::string fragmentShader = {
	"#version 450\n"
	"in highp vec4 vColor;\n"
	"out highp vec4 fColor;\n"
	"void main(){\n"
	"fColor = vColor;\n"
	"}"
};

AEOpenGLViewer::AEOpenGLViewer() : move_configuration(GLWindowMoveConfig::Freedom), view_configuration(GLWindowViewConfig::Perspective),
ortho_clipping(100.0f), ortho_width(150.0f), vertical_angle(45.0f), mousePrevPosition(QCursor::pos()) {
	setMouseTracking(true);
	initialize(800, 600);
}

AEOpenGLViewer::AEOpenGLViewer(GLWindowViewConfig viewConf, GLWindowMoveConfig moveConfig) : mousePrevPosition(QCursor::pos()) {
	move_configuration = moveConfig;
	view_configuration = viewConf;
	setMouseTracking(true);
	initialize(800, 600);
}

AEOpenGLViewer::AEOpenGLViewer(GLWindowViewConfig viewConf, GLWindowMoveConfig moveConfig, int width, int height) : mousePrevPosition(QCursor::pos()) {
	move_configuration = moveConfig;
	view_configuration = viewConf;
	setMouseTracking(true);
	initialize(width, height);
}

AEOpenGLViewer::~AEOpenGLViewer() {
	cleanGL();
	delete shader_program;
}

void AEOpenGLViewer::initialize(int width, int height) {
	QSurfaceFormat format;
	format.setRenderableType(QSurfaceFormat::OpenGL);
	format.setProfile(QSurfaceFormat::CoreProfile);
	format.setSwapInterval(1);
	setFormat(format);

	resize(QSize(width, height));
}

void AEOpenGLViewer::initializeGL() {
	// Initialize OpenGL Backend
	initializeOpenGLFunctions();

	//connect(this, SIGNAL(frameSwapped()), this, SLOT(update())); //Just wheter you want a continue rendering
	printContextInformation();

	// Set global information
	glEnable(GL_CULL_FACE);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	shader_program = new QOpenGLShaderProgram();
	shader_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader.c_str());
	shader_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader.c_str());
	shader_program->link();
	shader_program->bind();

	// Cache Uniform Locations
	u_modelToWorld = shader_program->uniformLocation("modelToWorld");
	u_worldToCamera = shader_program->uniformLocation("worldToCamera");
	u_cameraToView = shader_program->uniformLocation("cameraToView");

	for (size_t globjects_index = 0; globjects_index < objects.size(); ++globjects_index) {
		objects.at(globjects_index).vbo->create();
		objects.at(globjects_index).vbo->bind();
		objects.at(globjects_index).vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
		objects.at(globjects_index).vbo->allocate(objects.at(globjects_index).vertices.data(), objects.at(globjects_index).vertices.size() * sizeof(Vertex));

		objects.at(globjects_index).vao->create();
		objects.at(globjects_index).vao->bind();

		shader_program->enableAttributeArray(0);
		shader_program->enableAttributeArray(1);
		shader_program->setAttributeBuffer(0, GL_FLOAT, Vertex::positionOffset(), Vertex::PositionTupleSize, Vertex::stride());
		shader_program->setAttributeBuffer(1, GL_FLOAT, Vertex::colorOffset(), Vertex::ColorTupleSize, Vertex::stride());

		objects.at(globjects_index).vbo->release();
		objects.at(globjects_index).vao->release();
	}

	shader_program->release();

	is_initilized = true;
}

void AEOpenGLViewer::paintGL() {
	renderGL();
}

void AEOpenGLViewer::resizeGL(int width, int height) {
	float ratio = width / float(height);
	projection.setToIdentity();

	switch (view_configuration) {
	case Perspective:
		setPerspectiveProjection(width, height);
		break;
	case Orthogonal:
		setOrthogonalProjection(width, height);
		break;
	default:
		break;
	}
}

void AEOpenGLViewer::cleanGL() {
	makeCurrent();
	for (size_t i = 0; i < objects.size(); ++i) {
		objects.at(i).vao->destroy();
		objects.at(i).vbo->destroy();
		delete objects.at(i).vbo;
		delete objects.at(i).vao;
	}
	doneCurrent();
	objects.clear();
	camera.restore();

	update();
}

void AEOpenGLViewer::renderGL() {
	// Clear
	glClear(GL_COLOR_BUFFER_BIT);

	shader_program->bind();

	shader_program->setUniformValue(u_worldToCamera, camera.toMatrix());
	shader_program->setUniformValue(u_cameraToView, projection);
	{
		for (size_t globjects_index = 0; globjects_index < objects.size(); ++globjects_index) {
			objects.at(globjects_index).vao->bind();
			shader_program->setUniformValue(u_modelToWorld, objects.at(globjects_index).transform.toMatrix());

			glDrawArrays(objects.at(globjects_index).primitive_type, 0, objects.at(globjects_index).vertices.size());
			objects.at(globjects_index).vao->release();
		}
	}
	shader_program->release();
}

void AEOpenGLViewer::setProjection(int width, int height) {
	switch (view_configuration) {
	case Perspective:
		setPerspectiveProjection(width, height);
		break;
	case Orthogonal:
		setOrthogonalProjection(width, height);
		break;
	default:
		break;
	}
}

void AEOpenGLViewer::setOrthogonalProjection(int width, int height) {
	projection.setToIdentity();
	float ratio = width / float(height);

	if (width <= height)
		projection.ortho(-((ortho_width) / 2), ((ortho_width) / 2), -((ortho_width) / 2) / ratio, ((ortho_width) / 2) / ratio, -ortho_clipping, ortho_clipping);
	else
		projection.ortho(-((ortho_width) / 2) * ratio, ((ortho_width) / 2) * ratio, -((ortho_width) / 2), ((ortho_width) / 2), -ortho_clipping, ortho_clipping);
}

void AEOpenGLViewer::setPerspectiveProjection(int width, int height) {
	projection.setToIdentity();
	float ratio = width / float(height);

	projection.perspective(vertical_angle, ratio, 0.0f, 1000.0f);
}

void AEOpenGLViewer::prepareObjectInGPU(GLObject globject) {
	makeCurrent();
	shader_program->bind();
	{
		// Create Buffer (Do not release until VAO is created) (vbo)
		globject.vbo->create();
		globject.vbo->bind();
		globject.vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
		globject.vbo->allocate(globject.vertices.data(), globject.vertices.size() * sizeof(Vertex));

		// Create Vertex Array Object (vao)
		globject.vao->create();
		globject.vao->bind();

		shader_program->enableAttributeArray(0);
		shader_program->enableAttributeArray(1);
		shader_program->setAttributeBuffer(0, GL_FLOAT, Vertex::positionOffset(), Vertex::PositionTupleSize, Vertex::stride());
		shader_program->setAttributeBuffer(1, GL_FLOAT, Vertex::colorOffset(), Vertex::ColorTupleSize, Vertex::stride());

		// Release (unbind) all
		globject.vbo->release();
		globject.vao->release();
	}
	shader_program->release();
	doneCurrent();
}

void AEOpenGLViewer::setVertices(std::vector<Vertex> vertices, GLenum primitive) {

	GLObject object;
	object.vbo = new QOpenGLBuffer();
	object.vao = new QOpenGLVertexArrayObject();
	object.primitive_type = primitive;
	object.vertices = vertices;
	object.transform = Transform3D();

	objects.push_back(object);

	if (is_initilized) {
		prepareObjectInGPU(objects.at(objects.size() - 1));
		update();
	}
}

bool AEOpenGLViewer::toImage(QString img_name, int width, int height) {
	makeCurrent();

	// Information in order to restore later...
	int current_width = this->width();
	int current_height = this->height();

	setProjection(width, height);
	glViewport(0, 0, width, height);
	QOpenGLFramebufferObject fbo(width, height);
	fbo.bind();
	{
		renderGL();
	}
	fbo.release();
	QImage image(fbo.toImage());
	bool success = image.save(img_name);

	//Restore
	glViewport(0, 0, current_width, current_height);
	setProjection(current_width, current_height);

	doneCurrent();
	return success;
}

void AEOpenGLViewer::printContextInformation() {
	QString glType;
	QString glVersion;
	QString glProfile;

	// Get Version Information
	glType = (context()->isOpenGLES()) ? "OpenGL ES" : "OpenGL";
	glVersion = reinterpret_cast<const char*>(glGetString(GL_VERSION));

	// Get Profile Information
#define CASE(c) case QSurfaceFormat::c: glProfile = #c; break
	switch (format().profile()) {
		CASE(NoProfile);
		CASE(CoreProfile);
		CASE(CompatibilityProfile);
	}
#undef CASE

	// qPrintable() will print our QString w/o quotes around it.
	qDebug() << qPrintable(glType) << qPrintable(glVersion) << "(" << qPrintable(glProfile) << ")";
}

void AEOpenGLViewer::inputHandle(int key) {
	// Camera Transformation
	if (button_pressed) {
		static const float transSpeed = 10.0f;
		static const float rotSpeed = 0.5f;

		// Handle translations
		QVector3D translation;
		if (button_pressed) {
			switch (key) {
			case Qt::Key_W:
				camera.translate(camera.forward()*transSpeed);
				break;
			case Qt::Key_S:
				camera.translate(-camera.forward()*transSpeed);
				qDebug() << camera;
				break;
			case Qt::Key_A:
				camera.translate(-camera.right());
				break;
			case Qt::Key_D:
				camera.translate(camera.right());
				break;
			case Qt::Key_E:
				camera.translate(camera.up()*transSpeed);
				break;
			case Qt::Key_Q:
				camera.translate(-camera.up()*transSpeed);
				break;
			default:
				break;
			};
		}
		camera.translate(transSpeed * translation);
	}
}

void AEOpenGLViewer::update() {
	// Repaint
	QOpenGLWidget::update();
}

void AEOpenGLViewer::keyPressEvent(QKeyEvent *event) {
	if (event->isAutoRepeat())
		event->ignore();

	if (move_configuration == GLWindowMoveConfig::Freedom)
		inputHandle(event->key());

	update();
}

void AEOpenGLViewer::mousePressEvent(QMouseEvent *event) {
	if (event->button() == Qt::LeftButton) {
		button_pressed = true;
	}
}

void AEOpenGLViewer::mouseReleaseEvent(QMouseEvent *event) {
	if (event->button() == Qt::RightButton) {
		button_pressed = false;
	}
}

int AEOpenGLViewer::setProjectionWidth(float orthogonal_width) {
	if (view_configuration != GLWindowViewConfig::Orthogonal)
		return -1;

	this->ortho_width = orthogonal_width;
	setOrthogonalProjection(width(), height());
	update();
	return 0;
}

int AEOpenGLViewer::setProjectionClippingDepth(float orthogonal_clipping) {
	if (view_configuration != GLWindowViewConfig::Orthogonal)
		return -1;

	this->ortho_clipping = orthogonal_clipping;
	setOrthogonalProjection(width(), height());
	update();
	return 0;
}
