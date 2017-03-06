#include "Camera3d.h"
#include <QDebug>
#include <math.h>
#include <algorithm>

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

//Right hand method...
const QVector3D Camera3D::LocalForward(0.0f, 0.0f, -1.0f);
const QVector3D Camera3D::LocalUp(0.0f, 1.0f, 0.0f);
const QVector3D Camera3D::LocalRight(1.0f, 0.0f, 0.0f);

// Transform By (Add/Scale)
void Camera3D::translate(const QVector3D &dt) {
	m_dirty = true;
	m_translation += dt;
}

void Camera3D::rotate(const QQuaternion &dr) {
	m_dirty = true;
	m_rotation = dr * m_rotation;
}

// Transform To (Setters)
void Camera3D::setTranslation(const QVector3D &t) {
	m_dirty = true;
	m_translation = t;
}

void Camera3D::setRotation(const QQuaternion &r) {
	m_dirty = true;
	m_rotation = r;
}

// Accessors
const QMatrix4x4 &Camera3D::toMatrix() {
	// We’re building a matrix which will move us from being with respect of the world, 
	// to being with respect of the camera. We could form a matrix and then invert it;
	// or we could just build it inverted! <-
	if (m_dirty)
	{
		m_dirty = false;
		m_world.setToIdentity();
		m_world.rotate(m_rotation.conjugate());
		m_world.translate(-m_translation);
	}
	return m_world;
}

void Camera3D::restore() {
	m_dirty = true;
	setTranslation(QVector3D(0, 0, 0));
	setRotation(m_rotation.fromDirection(QVector3D(0, 0, 0), m_world.column(1).toVector3D()));
}

// Queries
QVector3D Camera3D::forward() const {
	return m_rotation.rotatedVector(LocalForward);
}

QVector3D Camera3D::forward(float n_steps) {
	return m_rotation.rotatedVector(LocalForward * n_steps);
}

QVector3D Camera3D::right() const {
	return m_rotation.rotatedVector(LocalRight);
}

QVector3D Camera3D::right(float n_steps) {
	return m_rotation.rotatedVector(LocalRight * n_steps);
}

QVector3D Camera3D::up() const {
	return m_rotation.rotatedVector(LocalUp);
}

QVector3D Camera3D::up(float n_steps) {
	return m_rotation.rotatedVector(LocalUp * n_steps);
}

// Qt Streams
QDebug operator<<(QDebug dbg, const Camera3D &transform) {
	dbg << "Camera3D {\n";
	dbg << "Position: <" << transform.getTranslation().x() << ", " << transform.getTranslation().y() << ", " << transform.getTranslation().z() << ">\n";
	dbg << "Rotation: <" << transform.getRotation().x() << ", " << transform.getRotation().y() << ", " << transform.getRotation().z() << " | " << transform.getRotation().scalar() << ">\n}";
	return dbg;
}

QDataStream &operator<<(QDataStream &out, const Camera3D &transform) {
	out << transform.m_translation;
	out << transform.m_rotation;
	return out;
}

QDataStream &operator >> (QDataStream &in, Camera3D &transform) {
	in >> transform.m_translation;
	in >> transform.m_rotation;
	transform.m_dirty = true;
	return in;
}