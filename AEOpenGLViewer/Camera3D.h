#pragma once

#include "Transform3D.h"

class Camera3D {

public:
	// Constants
	static const QVector3D LocalForward;
	static const QVector3D LocalUp;
	static const QVector3D LocalRight;

	// Constructors
	Camera3D();

	// Transform By (Add/Scale)
	void translate(const QVector3D &dt);
	void translate(float dx, float dy, float dz);
	void rotate(const QQuaternion &dr);
	void rotate(float angle, const QVector3D &axis);
	void rotate(float angle, float ax, float ay, float az);
	void restore();


	// Transform To (Setters)
	void setTranslation(const QVector3D &t);
	void setTranslation(float x, float y, float z);
	void setRotation(const QQuaternion &r);
	void setRotation(float angle, const QVector3D &axis);
	void setRotation(float angle, float ax, float ay, float az);

	// Accessors
	const QVector3D& getTranslation() const;
	const QQuaternion& getRotation() const;
	const QMatrix4x4& toMatrix();

	// Queries
	QVector3D forward() const;
	QVector3D forward(float n_steps);
	QVector3D right() const;
	QVector3D right(float n_steps);
	QVector3D up() const;
	QVector3D up(float n_steps);

private:
	bool m_dirty;
	QVector3D m_translation;
	QQuaternion m_rotation;
	QMatrix4x4 m_world;

	//m_word= |  right_x  up_x  forward_x  position_x  |
	//        |  right_y  up_y  forward_y  position_y  |
	//        |  right_z  up_z  forward_z  position_z  | 
	//        |     0      0        0          1       |

#ifndef QT_NO_DATASTREAM
	friend QDataStream &operator<<(QDataStream &out, const Camera3D &transform);
	friend QDataStream &operator >> (QDataStream &in, Camera3D &transform);
#endif
};

// Constructors
inline Camera3D::Camera3D() : m_dirty(true) {}

// Transform By (Add/Scale)
inline void Camera3D::translate(float dx, float dy, float dz) { translate(QVector3D(dx, dy, dz)); }
inline void Camera3D::rotate(float angle, const QVector3D &axis) { rotate(QQuaternion::fromAxisAndAngle(axis, angle)); }
inline void Camera3D::rotate(float angle, float ax, float ay, float az) { rotate(QQuaternion::fromAxisAndAngle(ax, ay, az, angle)); }

// Transform To (Setters)
inline void Camera3D::setTranslation(float x, float y, float z) { setTranslation(QVector3D(x, y, z)); }
inline void Camera3D::setRotation(float angle, const QVector3D &axis) { setRotation(QQuaternion::fromAxisAndAngle(axis, angle)); }
inline void Camera3D::setRotation(float angle, float ax, float ay, float az) { setRotation(QQuaternion::fromAxisAndAngle(ax, ay, az, angle)); }

// Accessors
inline const QVector3D& Camera3D::getTranslation() const { return m_translation; }
inline const QQuaternion& Camera3D::getRotation() const { return m_rotation; }

// Qt Streams
#ifndef QT_NO_DEBUG_STREAM
QDebug operator << (QDebug dbg, const Camera3D &transform);
#endif

#ifndef QT_NO_DATASTREAM
QDataStream &operator << (QDataStream &out, const Camera3D &transform);
QDataStream &operator >> (QDataStream &in, Camera3D &transform);
#endif
