#pragma once

#include "Vertex.h"
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointT;

/**
*	@class Data
*	@brief This class is used in order to control the world coordinate and transforms it in OpenGL Coordinate.
*/
class Data {
public:
	virtual int importData(QString filename) = 0;
	virtual int importData(QString filename, QVector3D color) = 0;
	virtual int importData(std::vector<PointT> &point_cloud_vector) = 0;
	virtual int importData(std::vector<PointT> &point_cloud_vector, QVector3D color) = 0;

	size_t getVertexSize();
	std::vector<Vertex> getVertices();
	void clear();

	/**
	*	@brief To get Vertex coordinate of a point. For correct operation, it is necessary to have imported the data previously.
	*	@param point	Euclidean coordinate of the point.
	*/
	virtual QVector3D getVertexCoordFromPoint(QVector3D point) = 0;
	/**
	*	@brief To get Vertex coordinate of a point. For correct operation, it is necessary to have imported the data previously.
	*	@param x	x-coordinate of the point.
	*	@param y	x-coordinate of the point.
	*	@param z	x-coordinate of the point.
	*/
	virtual QVector3D getVertexCoordFromPoint(float x, float y, float z) = 0;

protected:
	std::vector<Vertex> vertex_list;
	size_t scale_factor; //Scale used to convert double data to float, LAS File to PCD uses it.

	/**
	*	@brief Getting the name of a file. It will return the name of the file, without path.
	*	@param filename filename with path
	*/
	std::string getFilename(std::string filename);

	/**
	*	@brief Getting the path of a file. It will return the path of the file, without name.
	*	@param filename filename with path
	*/
	std::string getPath(std::string filename);

	/**
	*	@brief Transforms the data in Vertex in order to print in a OpenGL view. If you don't indicate a color,
	*		It will use the color represents in the pcl::PointXYZRGB (PointT of Data class).
	*	@param color	Color (RGB) used to paint. If it is not indicated, it will use the RGB of pcl::PointXYZRGB
	*/
	virtual void generateVertex() = 0;
	virtual void generateVertex(QVector3D color) = 0;
};

inline std::vector<Vertex> Data::getVertices() { return vertex_list; }
inline size_t Data::getVertexSize() { return sizeof(Vertex); }
inline std::string Data::getFilename(std::string filename) { return filename.substr(filename.find_last_of("/\\") + 1); }
inline std::string Data::getPath(std::string filename) { return filename.substr(0, filename.find_last_of("/\\")); }
inline void Data::clear() { if (vertex_list.size() > 0)	vertex_list.clear(); }