#pragma once

#ifndef VERTEX_H
#define VERTEX_H

#include <QVector3D>
#include "aeopenglviewer_global.h"

class AEOPENGLVIEWER_EXPORT Vertex {
public:
	Vertex();
	Vertex(const QVector3D &position);
	Vertex(const QVector3D &position, const QVector3D &color);
	~Vertex();

	const QVector3D& getPosition() const;
	const QVector3D& getColor() const;
	void setPosition(const QVector3D& position);
	void setColor(const QVector3D& color);

	// OpenGL Helpers
	static const int PositionTupleSize = 3;
	static const int ColorTupleSize = 3;
	static Q_DECL_CONSTEXPR int positionOffset();
	static Q_DECL_CONSTEXPR int colorOffset();
	static Q_DECL_CONSTEXPR int stride();
private:
	QVector3D position;
	QVector3D color;
};

inline const QVector3D& Vertex::getPosition() const { return position; }
inline const QVector3D& Vertex::getColor() const { return color; }
void inline Vertex::setPosition(const QVector3D& position) { this->position = position; }
void inline Vertex::setColor(const QVector3D& color) { this->color = color; }

inline int Vertex::positionOffset() { return offsetof(Vertex, position); }
inline int Vertex::colorOffset() { return offsetof(Vertex, color); }
inline int Vertex::stride() { return sizeof(Vertex); }

#endif
