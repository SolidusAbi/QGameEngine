#pragma once
#ifndef GLWINDOW_H
#define GLWINDOW_H

#include <QtWidgets/QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QTimer>

#include "aeopenglviewer_global.h"
#include "Transform3D.h"
#include "Camera3D.h"
#include "Vertex.h"

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
	Q_OBJECT

	typedef struct {
		QOpenGLBuffer *vbo;
		QOpenGLVertexArrayObject *vao;
		GLenum primitive_type;
		std::vector<Vertex> vertices;
		Transform3D transform;
	} GLObject;

public:
	AEOpenGLViewer();
	AEOpenGLViewer(GLWindowViewConfig viewConf, GLWindowMoveConfig moveConfig);
	AEOpenGLViewer(GLWindowViewConfig viewConf, GLWindowMoveConfig moveConfig, int width, int height);
	~AEOpenGLViewer();

	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();

	/**
	*	@brief Clean the OpenGL visualization
	*/
	void cleanGL();

	/**
	*	@brief vector with vertex of OpenGL Window.
	*	@param vertices		vector with vertices.
	*/
	void setVertices(std::vector<Vertex> vertices);

	/**
	*	@brief vector with vertex of OpenGL Window.
	*	@param vertices		vector with vertices.
	*	@param primitive	type of vertex (GL_POINTS, GL_LINES...)
	*/
	void setVertices(std::vector<Vertex> vertices, GLenum primitive);

	/**
	*	@brief set the clipping value with is used in order to indicate near and far plane in orthogonal view.
	*	@paramclippingValue
	*/
	void setClippingValue(float clippingValue);

	/**
	*	@brief Save the OpenGL render in a image file. The default size is 800x600.
	*	@param img		name of the image file, including the path
	*/
	bool toImage(QString img);

	/**
	*	@brief Save the OpenGL render in a image file.
	*	@param img		name of the image file, including the path
	*	@param width	width of the image file
	*	@param height	height of the image file
	*/
	bool toImage(QString img_name, int width, int height);

	/**
	*	@brief Setting the projection zoom in a Orthogonal view adjusted with the width of the projection.
	*	@param	orthogonal_width	width of the orthogonal projection matrix
	*	@return -1 (It is not a orthogonal view) and 0 (Success)
	*/
	int setProjectionWidth(float orthogonal_width);

	/**
	*	@brief Setting the projection depth in a Orthogonal view adjusted with the clipping plane of the projection in the Z-axis.
	*	@param orthogonal_clipping	width of the orthogonal projection matrix
	*	@return -1 (It is not a orthogonal view) and 0 (Success)
	*/
	int setProjectionClippingDepth(float orthogonal_clipping);

protected slots:
	/**
	*	@brief slot used to update the view.
	*/
	void update();

protected:
	// Shader Information
	QMatrix4x4 projection; //projection's matrix
	Camera3D camera; //camera's matrix contains the world move to camera

					 // General Information of the Window
	GLWindowViewConfig view_configuration;
	GLWindowMoveConfig move_configuration;

	// Orthogonal Visualization Information
	float ortho_clipping = 100.0f; //This attribute is used to indicate near and far plane in orthogonal view
	float ortho_width = 150.0f; // Used to indicate the projection zoom

								//Perspective Visualization Information
	float vertical_angle = 45.0f; //Range of vision in perspective vision

								  // Visualization Information
	std::vector<GLObject> objects;

	/**
	*	@Brief Handle of the input command, this method must to be implemented
	*		in each class that inherits from this one.
	*/
	virtual void inputHandle(int key);

	/**
	*	@brief Prepare the projection matrix
	*
	*	@param width	width of the window
	*	@param height	height of the window
	*/
	void setProjection(int width, int height);

	/**
	*	@brief Prepare the projection matrix with orthogonal view
	*	@param width	width of the window
	*	@param height	height of the window
	*/
	void setOrthogonalProjection(int width, int height);

	/**
	*	@brief Prepare the projection matrix with perspective view
	*	@param width	width of the window
	*	@param height	height of the window
	*/
	void setPerspectiveProjection(int width, int height);

	/**
	*	@brief function used to allocate objects in memory of GPU
	*/
	void prepareObjectInGPU(GLObject globject);


	//Prueba (Hay que mejorarlo)
	bool button_pressed = false;
	QPoint mousePrevPosition;

	void keyPressEvent(QKeyEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
private:
	// OpenGL State Information
	QOpenGLShaderProgram *shader_program;
	bool is_initilized = false;

	// Shader Information 
	int u_modelToWorld;
	int u_worldToCamera;
	int u_cameraToView;

	// Private Helpers
	void printContextInformation();
	void initialize(int width, int height);
	void renderGL();
};

inline void AEOpenGLViewer::setVertices(std::vector<Vertex> vertices) { setVertices(vertices, GL_POINTS); }
inline void AEOpenGLViewer::setClippingValue(float clipping_value) { ortho_clipping = clipping_value; }
inline bool AEOpenGLViewer::toImage(QString img) { return toImage(img, 800, 600); }

#endif // GLWINDOW_H

