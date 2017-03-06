#include "Vertex.h"

Vertex::Vertex() {}

Vertex::Vertex(const QVector3D &position) :position(position), color(QVector3D(1, 1, 1)) {} //White as default color

Vertex::Vertex(const QVector3D &position, const QVector3D &color) : position(position), color(color) {}

Vertex::~Vertex() {}
