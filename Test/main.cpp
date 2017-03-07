#include <AEOpenGLHandler.h>
#include <Vertex.h>
#include "Test.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	AEOpenGLHandler handler(AEOpenGLHandlerView::AnomalyWindow, 800, 600);
	Test w;
	w.show();
	return a.exec();
}
