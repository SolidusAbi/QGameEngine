#pragma once
#ifndef AEOPENGLHANDLE_H
#define AEOPENGLHANDLE_H

#include <QString>

#include "aeopenglviewer_global.h"
#include "AEOpenGLViewer.h"
#include "Data.h"
#include "Vertex.h"

enum AEOpenGLHandlerView {
	AnomalyWindow,
	Default
};

enum AnomalyView {
	top,
	cross,
	longitudinal,
	none
};


class Data;
class AEOpenGLViewer;

/**
*	@class	AEOpenGLHandle
*	@brief	This class is a controller in order to handle the GLWindow class. It is not necessary
*		to use but it is recommender.
*/
class AEOPENGLVIEWER_EXPORT AEOpenGLHandler {

	enum AEOpenGLHandlerState {
		Created,
		Cleansed,
		DataImported,
		VertexImported,
		Rendering
	};

	enum AEOpenGLHandlerData {
		LIDAR
	};

public:
	AEOpenGLHandler();
	AEOpenGLHandler(AEOpenGLHandlerView view, int width, int height);
	~AEOpenGLHandler();

	/**
	*	@brief setting the data.
	*	@param data	pointer to a Data object
	*/
	void setData(Data *data);

	/**
	*	@brief Importing data from a PCD or LAS file.
	*	@param filename	file from will be data imported
	*	@param color	Color (RGB) in order to paint the points
	*/
	int importDataFromFile(QString filename);
	int importDataFromFile(QString filename, QVector3D color);

	/**
	*	@brief Importing points from a vector of pcl::PointT
	*	@param vector	vector of pcl::PointT
	*	@param color	Color (RGB) in order to paint the points
	*/
	int importDataFromVector(std::vector<PointT> &vector);
	int importDataFromVector(std::vector<PointT> &vector, QVector3D color);

	/**
	*	@brief Setting the projection line used in order to generates multiple views in AnomalyWindow
	*	@param ori
	*/
	int setProjectionLine(QVector3D ori, QVector3D dst);
	int show();
	void clean();

	/**
	*	@brief Setting the projection zoom in a orthogonal projection viewer
	*	@param projection_width	width of the projection
	*/
	int setProjectionWidth(float projection_width);

	/**
	*	@brief Setting the projection clipping depth in a orthogonal projection viewer
	*	@param projection_depth	depth of the projection
	*/
	int setProjectionDepth(float projection_depth);

	/**
	*	@brief Setting the view, this function only works with a Anomaly Viewer.
	*	@param type		View type (top, cross, longitudinal or none)
	*	@param center	point where will be localed in (0,0) OpenGL coordinate
	*/
	int setView(AnomalyView type);
	int setView(AnomalyView type, QVector3D center);

	/**
	*	@brief To paint multiple vertices in the OpenGL Widget
	*	@param point	vertices to paint
	*	@param color	color with which to paint the vertices
	*	@primitive_type	GLenum(GL_POINTS, GL_LINES...)
	*/
	void paint(std::vector<QVector3D> &point, QVector3D &color, GLenum primitive_type);
private:
	AEOpenGLHandlerState state;
	AEOpenGLHandlerData data_type;
	AEOpenGLHandlerView view_type;

	Data *data;
	AEOpenGLViewer *window;

	//Private Helpers
	void initialize(int width, int height);
	int importVertex();
};

inline void AEOpenGLHandler::setData(Data *data) { clean(); this->data = data; state = AEOpenGLHandlerState::DataImported; }

inline int AEOpenGLHandler::importDataFromFile(QString filename) { return importDataFromFile(filename, QVector3D(-1, -1, -1)); }
inline int AEOpenGLHandler::setProjectionWidth(float projection_width) { return window->setProjectionWidth(projection_width); }
inline int AEOpenGLHandler::setProjectionDepth(float projection_deep) { return window->setProjectionClippingDepth(projection_deep); }

#endif //AEOPENGLHANDLE_H
