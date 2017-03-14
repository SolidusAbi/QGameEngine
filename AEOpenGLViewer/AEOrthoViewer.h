#pragma once
#ifndef ANOMALYGLWINDOW_H
#define ANOMALYGLWINDOW_H

#include <QWheelEvent>
#include <QMouseEvent>

#include "Vertex.h"
#include "AEOpenGLViewer.h"

/**
*	@class AnomalyGLWindow
*	@brief QOpenGLWindow used in Anomaly Report. It is designed to use AEROLASER LIDAR data acquired from helicopter.
*/
class AEOPENGLVIEWER_EXPORT AEOrthoViewer : public AEOpenGLViewer {

public:
	AEOrthoViewer();
	AEOrthoViewer(int width, int height);
	AEOrthoViewer(Vertex line_coord_0, Vertex line_coord_1);
	AEOrthoViewer(Vertex line_coord_0, Vertex line_coord_1, int window_width, int window_height);
	~AEOrthoViewer();

	void setProjectionLine(Vertex coord_0, Vertex coord_1);
	std::vector<Vertex> getProjectionLineVertices();

	template <typename T = float> void setAnomalyPoint(T x, T y, T z);
	void setAnomalyPoint(QVector3D anomaly_point);
	QVector3D anomalyPoint();

	/**
	*	@brief To generate a top view with the anomaly in the Window
	*	@param x X coordinate of the anomaly
	*	@param y Y coordinate of the anomaly
	*	@param z Z coordinate of the anomaly
	*
	*	Anomaly Reports uses various types of views, this function is used in order to generate
	*	the top view. There are more type of view implemented in the class (longitudinalView() and crossView()).
	*	The TOP View is a default view in AEROLASER data, LAS data acquired from helicopter, this function only
	*	moves the model in order to center it in the anomaly.
	*/
	template <typename T = float> int topView(T x, T y, T z);
	int topView(QVector3D anomaly_point);
	int topView();

	/**
	*	@brief To generate a cross view with the anomaly in the Window
	*	@param x X coordinate of the anomaly
	*	@param y Y coordinate of the anomaly
	*	@param z Z coordinate of the anomaly
	*
	*	Anomaly Reports uses various types of views, this function is used in order to generate
	*	the cross view. There are more type of view implemented in the class (longitudinalView() and topView()).
	*/
	template <typename T = float> int crossView(T x, T y, T z);
	int crossView(QVector3D anomaly_point);
	int crossView();

	/**
	*	@brief To generate a longitudinal view with the anomaly in the Window
	*	@param x X coordinate of the anomaly
	*	@param y Y coordinate of the anomaly
	*	@param z Z coordinate of the anomaly
	*
	*	Anomaly Reports uses various types of views, this function is used in order to generate
	*	the longitudinal view. There are more type of view implemented in the class (crossView() and topView()).
	*/
	template <typename T = float> int longitudinalView(T x, T y, T z);
	int longitudinalView(QVector3D anomaly_point);
	int longitudinalView();

	/**
	*	@brief Clean the OpenGL visualization
	*/
	void cleanGL();
private:
	//View information
	AnomalyView anomaly_view;

	// Line Information
	bool lineDataPrepare();
	std::vector<Vertex> projection_line_vertices;	//line coordinates are a XY point, Z-axis is irrelevant.
	float line_slope;
	double line_angle;					//Inclination angle of the line with respect to the X-axis, in degrees

										// Anomaly Information 
	QVector3D anomaly_point;			//Anomaly point closest to the line

	/**
	*	@brief reseting the camera, camera will be centered in the anomaly point.
	*/
	void reset(AnomalyView view = AnomalyView::none);


	/**
	*	@Brief Handle of the input command, this method must to be implemented
	*		in each class that inherits from this one.
	*/
	void inputHandle(int key);

	// Private Helpers
	void settingLongitudinalViewCamera();
	void settingCrossViewCamera();

protected:
	void wheelEvent(QWheelEvent *event);
	void mouseMoveEvent(QMouseEvent * event);
};

inline bool AEOrthoViewer::lineDataPrepare() { return (projection_line_vertices.empty() ? false : true); }

template <typename T> inline void AEOrthoViewer::setAnomalyPoint(T x, T y, T z) { setAnomalyPoint(QVector3D(x, y, z)); }
inline QVector3D AEOrthoViewer::anomalyPoint() { return anomaly_point; }

inline std::vector<Vertex> AEOrthoViewer::getProjectionLineVertices() { return projection_line_vertices; }

template <typename T> inline int AEOrthoViewer::topView(T x, T y, T z) { return topView(QVector3D(x, y, z)); }
template <typename T> inline int AEOrthoViewer::crossView(T x, T y, T z) { return crossView(QVector3D(x, y, z)); }
template <typename T> inline int AEOrthoViewer::longitudinalView(T x, T y, T z) { return longitudinalView(QVector3D(x, y, z)); }

#endif // ANOMALYGLWINDOW_H
