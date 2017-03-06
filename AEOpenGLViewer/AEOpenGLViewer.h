#pragma once

#include <QtWidgets/QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>


#include "aeopenglviewer_global.h"

enum GLWindowViewConfig {
	Perspective,
	Orthogonal
};

enum GLWindowMoveConfig {
	Freedom,
	Fixed
};

class QOpenGLShaderProgram;

/**
*	@class AEOpenGLViewer
*	@brief This viewer has  used in order to show point clouds.
*/

class AEOPENGLVIEWER_EXPORT AEOpenGLViewer : public QOpenGLWidget, protected QOpenGLFunctions {

public:
	AEOpenGLViewer();
};
