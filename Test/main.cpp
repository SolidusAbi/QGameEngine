#include <AEOpenGLHandler.h>
#include <Vertex.h>
#include "Test.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Test w;
	w.laPrueba();
	//w.print2();
	w.show();
	return a.exec();
}
