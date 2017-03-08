#include <pcl/io/pcd_io.h>

#include "DataLIDAR.h"

DataLIDAR::DataLIDAR() : point_cloud(new pcl::PointCloud<PointT>) {}

DataLIDAR::~DataLIDAR() { clear(); }

int DataLIDAR::importData(QString filename, QVector3D color) {
	QFileInfo file_info(filename);
	if (!file_info.exists())
		return -2;

	QString extension = file_info.completeSuffix();
	if (QString::compare(extension, "pcd", Qt::CaseInsensitive) == 0)
		return importPCDFile(filename.toStdString(), color);

	if (QString::compare(extension, "las", Qt::CaseInsensitive) == 0)
		return importLASFile(filename.toStdString());

	return -1;
}

int DataLIDAR::importData(std::vector<PointT> &point_cloud_vector) {
	double tmpSumX = 0, tmpSumY = 0, tmpSumZ = 0;
	for (size_t point_cloud_index = 0; point_cloud_index < point_cloud_vector.size(); ++point_cloud_index) {
		PointT tmp = point_cloud_vector.at(point_cloud_index);
		point_cloud->points.push_back(tmp);

		/*Temporal*/
		tmpSumX += tmp.x;
		tmpSumY += tmp.y;
		tmpSumZ += tmp.z;
	}

	center_xyz.push_back(tmpSumX / point_cloud->size());
	center_xyz.push_back(tmpSumY / point_cloud->size());
	center_xyz.push_back(tmpSumZ / point_cloud->size());

	generateVertex();

	return 0;
}

int DataLIDAR::importData(std::vector<PointT>& point_cloud_vector, QVector3D color) {
	double tmpSumX = 0, tmpSumY = 0, tmpSumZ = 0;
	for (size_t point_cloud_index = 0; point_cloud_index < point_cloud_vector.size(); ++point_cloud_index) {
		PointT tmp = point_cloud_vector.at(point_cloud_index);
		point_cloud->points.push_back(tmp);

		/*Temporal*/
		tmpSumX += tmp.x;
		tmpSumY += tmp.y;
		tmpSumZ += tmp.z;
	}

	center_xyz.push_back(tmpSumX / point_cloud->size());
	center_xyz.push_back(tmpSumY / point_cloud->size());
	center_xyz.push_back(tmpSumZ / point_cloud->size());

	generateVertex(color);

	return 0;
}

int DataLIDAR::importPCDFile(std::string filename, QVector3D color) {
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *point_cloud) == -1) {
		PCL_ERROR("Couldn't read file\n");
		return -1;
	}

	std::cout << "Loaded "
		<< point_cloud->width * point_cloud->height
		<< " data points from " << filename
		<< std::endl;

	double tmpSumX = 0, tmpSumY = 0, tmpSumZ = 0;
	for (size_t point_cloud_index = 0; point_cloud_index < point_cloud->size(); ++point_cloud_index) {
		pcl::PointXYZRGB tmp = point_cloud->at(point_cloud_index);

		//Temporal to generate a center value whenever is necessary
		tmpSumX += tmp.x;
		tmpSumY += tmp.y;
		tmpSumZ += tmp.z;
	}

	if (center_xyz.empty() || center_xyz.at(0) == CenterUnknownValue) {
		//Generate a center value
		center_xyz.push_back(tmpSumX / point_cloud->size());
		center_xyz.push_back(tmpSumY / point_cloud->size());
		center_xyz.push_back(tmpSumZ / point_cloud->size());
	}

	if (color == QVector3D(-1, -1, -1))
		generateVertex();
	else
		generateVertex(color);

	return 0;
}

int DataLIDAR::importLASFile(std::string filename) {
	QList<AE_pointXYZRGB> pointList;
	std::string pcd_filename = getFilename(filename);
	pcd_filename = pcd_filename.substr(0, pcd_filename.size() - 3).append("pcd");

	AeLasReader *lasReader;

	try {
		lasReader = new AeLasReader(QString(filename.c_str()));
	}
	catch (std::exception) {
		std::cout << "Cannot read LAS File" << std::endl;
		return -1;
	}

	AePointDataRecordSecuentialIterator iterator = lasReader->pointDataRecordSecuentialIterator();

	int percentage = 0;
	double tmpSumX = 0;
	double tmpSumY = 0;
	double tmpSumZ = 0;
	for (iterator; !iterator.atEnd(); ++iterator) {
		AePointRecordReader pointRecordReader = lasReader->getPointRecord();
		double nx = pointRecordReader.getAbsoluteX();
		double ny = pointRecordReader.getAbsoluteY();
		double nz = pointRecordReader.getAbsoluteZ();

		unsigned short nr = 0, ng = 255, nb = 0;
		if (lasReader->getHeader().getPointDataFormatId() == 3) {
			unsigned short nr = pointRecordReader.getColorRed() * 255 / 65535; //65535 por norma de los LAS...
			unsigned short ng = pointRecordReader.getColorGreen() * 255 / 65535;
			unsigned short nb = pointRecordReader.getColorBlue() * 255 / 65535;
		}
		AE_pointXYZRGB pointT(nx, ny, nz, nr, ng, nb);
		pointList.append(pointT);


		tmpSumX += nx;
		tmpSumY += ny;
		tmpSumZ += nz;

		pcl::PointXYZRGB tmp(nr, ng, nb);
		tmp.x = nx*scale_factor;
		tmp.y = ny*scale_factor;
		tmp.z = nz*scale_factor;

		point_cloud->points.push_back(tmp);

		if (percentage != iterator.percentage()) {
			qDebug() << iterator.percentage() << "%" << "\r";
			percentage = iterator.percentage();
		}
	}
	lasReader->close();
	delete lasReader;

	point_cloud->width = point_cloud->points.size();
	point_cloud->height = 1;
	pcl::io::savePCDFile(pcd_filename, *point_cloud, true);

	if (center_xyz.empty() || center_xyz.at(0) == CenterUnknownValue) {
		//Generate a center value
		center_xyz.push_back(tmpSumX / pointList.size());
		center_xyz.push_back(tmpSumY / pointList.size());
		center_xyz.push_back(tmpSumZ / pointList.size());
	}

	generateVertex();

	return 0;
}

void DataLIDAR::generateVertex() {
	if (point_cloud->size() <= 0 || point_cloud->points.size() <= 0)
		return;

	for (size_t point_index = 0; point_index < point_cloud->points.size(); ++point_index) {
		PointT point = point_cloud->points.at(point_index);
		QVector3D coord((float)(point.x - center_xyz[0]), (float)(point.y - center_xyz[1]), (float)(point.z - center_xyz[2]));
		QVector3D color((float)point.r / 255, (float)point.g / 255, (float)point.b / 255);
		vertex_list.push_back(Vertex(coord, color));
	}
}

void DataLIDAR::generateVertex(QVector3D color) {
	if (point_cloud->size() <= 0 || point_cloud->points.size() <= 0)
		return;

	for (size_t point_index = 0; point_index < point_cloud->points.size(); ++point_index) {
		PointT point = point_cloud->points.at(point_index);
		QVector3D coord((float)(point.x - center_xyz[0]), (float)(point.y - center_xyz[1]), (float)(point.z - center_xyz[2]));
		vertex_list.push_back(Vertex(coord, color));

		if (point_index > 4000000)
			break;
	}
}

QVector3D DataLIDAR::getVertexCoordFromPoint(float x, float y, float z) {
	if (!center_xyz.empty() && center_xyz.at(0) != CenterUnknownValue)
		return QVector3D((float)(x - center_xyz[0]), (float)(y - center_xyz[1]), (float)(z - center_xyz[2]));
	else
		return QVector3D(x, y, z);
}

void DataLIDAR::clear() {
	point_cloud->clear();
	if (!center_xyz.empty())
		center_xyz.clear();

	center_xyz = { CenterUnknownValue, CenterUnknownValue, CenterUnknownValue };

	Data::clear();
}