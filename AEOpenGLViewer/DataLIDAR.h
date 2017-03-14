#pragma once

#include <limits>
#include <QtCore>
#include <Vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Data.h"
#include "Vertex.h"
#include "AeLas/aelas.h"
#include "AE_DataTypes/ae_datatypes.h"

//Normalize
//z_i=x_o-min(x)/(max(x)-min(x)) //Maybe a future implementation

static const float CenterUnknownValue = std::numeric_limits<float>::min();

/**
*	@Class DataLIDAR
*	This class is used to represent LIDAR data in a OpenGL view. It contains a pcl::PointCloud wich contains the
*	LIDAR points and it converts them in OpenGL Vertex.
*/
class AEOPENGLVIEWER_EXPORT DataLIDAR : public Data {
public:
	DataLIDAR();
	~DataLIDAR();

	/**
	*	@brief Data file can be a LAS file (.las) or PCD file (.pcd)
	*	@param filename	path and name of the file
	*	@param color	Color (RGB) used to paint. If it is not indicated, it will use the RGB of pcl::PointXYZRGB
	*	@return			'file does not exist' (-2), 'Cannot read file' (-1), 'Success' (0)
	*/
	int importData(QString filename);
	int importData(QString filename, QVector3D color);

	/**
	*	@brief Import data from a vector with pcl points
	*	@param point_cloud_vector	This vector contains the pcl points
	*	@param color				Color (RGB) used to paint. If it is not indicated, it will use the RGB of pcl::PointXYZRGB
	*/
	int importData(std::vector<PointT> &point_cloud_vector);
	int importData(std::vector<PointT> &point_cloud_vector, QVector3D color);


	QVector3D getVertexCoordFromPoint(QVector3D point);
	QVector3D getVertexCoordFromPoint(float x, float y, float z);

	void clear();

private:
	pcl::PointCloud<PointT>::Ptr point_cloud;
	const uint scale_factor = 1; //Scale used to convert LAS data (double) to PCD (float)
	std::vector<float> center_xyz{ CenterUnknownValue, CenterUnknownValue, CenterUnknownValue }; //Point which is located in the OpenGL coordinate (0, 0)

	//This function is used to generate vertex from point cloud data
	void generateVertex();
	void generateVertex(QVector3D color);

	// Used to import from files.
	int importPCDFile(std::string filename, QVector3D color);
	int importLASFile(std::string filename);
};

inline int DataLIDAR::importData(QString filename) { return importData(filename, QVector3D(-1, -1, -1)); }
inline QVector3D DataLIDAR::getVertexCoordFromPoint(QVector3D point) { return getVertexCoordFromPoint(point.x(), point.y(), point.z()); }

